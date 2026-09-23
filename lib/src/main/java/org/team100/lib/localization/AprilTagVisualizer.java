package org.team100.lib.localization;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.TrailingHistory;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.util.struct.StructBuffer;

/**
 * Listens for camera updates, uses the robot's current pose estimate to paint
 * the tag's "apparent position" on the Field2d widget, and in AdvantageScope
 * using the Vision Target feature.
 * 
 * Each apparent tag position should match the actual position, if the cameras
 * are calibrated correctly, and if the camera/robot clocks are in sync.
 * 
 * This is kinda useful for debugging, but it's not *that* useful. It used to be
 * part of AprilTagCornerRobotLocalizer.
 */
public class AprilTagVisualizer extends CameraReader<BlipWithCorners> {
    private static final boolean DEBUG = false;
    /** Maximum age of the sights we publish for diagnosis. */
    private static final double HISTORY_DURATION = 1.0;
    private final PoseFromCorners m_estimator;
    private final StateSampler m_history;
    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final TrailingHistory<Pose3d> m_allTags;
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    private final DoubleArrayLogger m_log_allTags;
    private final DoubleLogger m_log_tag_error;

    public AprilTagVisualizer(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            StateSampler history,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        super(parent, "vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
        LoggerFactory log = parent.type(this);
        m_history = history;
        m_estimator = new PoseFromCorners();
        m_alliance = alliance;
        m_layout = layout;
        m_allTags = new TrailingHistory<>();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_log_allTags = fieldLogger.doubleArrayLogger(Level.DEBUG, "all tags");
        m_log_tag_error = log.doubleLogger(Level.DEBUG, "tag error");
    }

    /**
     * Clean the history, relative to the current moment.
     * 
     * Previously, eviction only occurred when the robot could see something.
     */
    @Override
    protected void beginUpdate() {
        double deadline = Takt.get() - HISTORY_DURATION;
        m_allTags.evict(deadline);
    }

    @Override
    protected void perValue(Camera camera, BlipWithCorners[] blips) {
        if (!Experiments.INSTANCE.enabled(Experiment.ShowTags))
            return;
        Transform3d cameraOffset = Offset.get(camera).offset();
        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            if (DEBUG)
                System.out.println("no alliance!");
            return;
        }
        Alliance alliance = optAlliance.get();

        for (int i = 0; i < blips.length; ++i) {
            BlipWithCorners blip = blips[i];

            // Camera-to-tag.
            Transform3d cameraToTag = tagInCamera(camera, blip);

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldOpt = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldOpt.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from JSON map.
            Pose3d tagInField = tagInFieldOpt.get();

            // Estimate the tag pose in the field frame.
            double blipTimeSec = (double) blip.getTimestamp() / 1e6;
            Pose2d samplePose = sample(blipTimeSec);
            Pose3d estimatedTagInField = estimatedTagInField(cameraOffset, samplePose, cameraToTag);
            m_allTags.add(blipTimeSec, estimatedTagInField);
            logTagError(tagInField, estimatedTagInField);

            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
                if (DEBUG)
                    System.out.println("Drop update, vision is off.");
                continue;
            }
        }
    }

    /** * Show the tags on the Field2d widget and AdvantageScope. */
    @Override
    protected void finishUpdate() {
        m_pub_tags.set(m_allTags.getAll().toArray(new Pose3d[0]));
        m_log_allTags.log(
                () -> m_allTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), x.toPose2d().getRotation().getDegrees())).toArray());
    }

    /** Log the norm of the translational error of the tag. */
    private void logTagError(Pose3d tagInField, Pose3d estimatedTagInField) {
        Transform3d tagError = tagInField.minus(estimatedTagInField);
        m_log_tag_error.log(() -> tagError.getTranslation().getNorm());
    }

    /** Sample the history at the frame timestamp. */
    private Pose2d sample(double timestamp) {
        // Note this pulls from the *old history*, not the *odometry-updated history*,
        // because we don't care about the latest odometry update.
        //
        // Because the camera delay is much more than the odometry delay, we're always
        // trying to write history from several cycles ago (followed by replay). It's ok
        // for new odometry to be the last thing.
        return m_history.get(timestamp).pose();
    }

    /**
     * Use the pose sample, camera offset, and tag-in-camera transform to estimate
     * the tag pose in the field frame.
     */
    private Pose3d estimatedTagInField(
            Transform3d cameraOffset, Pose2d pose, Transform3d tagInCamera) {
        // Field-to-robot
        Pose3d pose3d = new Pose3d(pose);
        // Field-to-robot plus robot-to-camera = field-to-camera
        Pose3d cameraPose = pose3d.transformBy(cameraOffset);
        // Given the historical pose, where do we think the tag is?
        return cameraPose.transformBy(tagInCamera);
    }

    /**
     * Camera-to-tag, as it appears in the camera frame.
     * The raw pose in the blip is "z-forward" like the camera.
     * This returns "x-forward" like the robot.
     */
    private Transform3d tagInCamera(Camera camera, BlipWithCorners blip) {
        float[] corners = blip.getCorners();
        double[] dCorners = new double[corners.length];
        for (int i = 0; i < corners.length; ++i) {
            dCorners[i] = corners[i];
        }
        Transform3d zFwd = m_estimator.pose(camera, dCorners);
        return GeometryUtil.zForwardToXForward(zFwd);
    }

}
