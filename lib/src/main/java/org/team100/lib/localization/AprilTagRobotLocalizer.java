package org.team100.lib.localization;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Takt;
import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VisionNoise;
import org.team100.lib.util.TrailingHistory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Extracts robot pose estimates from camera observations of AprilTags.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagRobotLocalizer extends CameraReader<Blip> {
    private static final boolean DEBUG = false;

    /** Maximum age of the sights we publish for diagnosis. */
    private static final double HISTORY_DURATION = 1.0;

    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;

    private final DoubleFunction<ModelSE2> m_history;
    private final VisionUpdater m_visionUpdater;
    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;

    /**
     * The apparent position of tags we see: this can be shown in AdvantageScope
     * using the Vision Target feature. The apparent position should match the
     * actual position, if the cameras are calibrated correctly. Note this involves
     * matching the frame timestamp with the pose history timestamp, so if the blip
     * source timestamp is wrong (as it is at the moment in the simulated tag
     * detector) then these positions will be a little bit wrong.
     */
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    /** Just tags we use for pose estimation. */
    private final StructArrayPublisher<Pose3d> m_pub_used_tags;
    /** Logging the same thing for the Glass Field2d widget, for simulation. */
    private final DoubleArrayLogger m_log_allTags;
    private final DoubleArrayLogger m_log_usedTags;

    /**
     * The pose we derive from each sighting, so we can see it in AdvantageScope's
     * map, which can't understand our usual Pose2dLogger's output.
     */
    private final StructPublisher<Pose2d> m_pub_pose;

    // LOGGERS
    private final LoggerFactory m_log_cameraToTag_factory;
    private final LoggerFactory m_log_robotToTag_factory;

    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;
    private final DoubleLogger m_log_tag_error;
    private final Pose2dLogger m_log_pose;

    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

    /**
     * Accumulates all tags we receive in each cycle, whether we use them or not.
     */
    private final TrailingHistory<Pose3d> m_allTags;
    /**
     * Just the tags we use for pose estimation, i.e. not ones that are too far
     * away.
     */
    private final TrailingHistory<Pose3d> m_usedTags;

    private final Map<String, Transform3dLogger> m_log_cameraToTag;
    private final Map<String, Transform3dLogger> m_log_tagInRobot;

    /**
     * Remember the previous vision-based pose estimate, so we can measure the
     * distance between consecutive updates, and ignore too-far updates.
     */
    private Pose2d m_prevPose;

    /**
     * Use tags closer than this. Ignore tags further than this.
     */
    private double m_heedRadiusM;

    /**
     * @param parent        logger
     * @param layout        map of apriltags
     * @param history       f(timestamp) = swerve state, use SwerveModelHistory.
     * @param visionUpdater mutates history
     */
    public AprilTagRobotLocalizer(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            DoubleFunction<ModelSE2> history,
            VisionUpdater visionUpdater,
            Supplier<Optional<Alliance>> alliance) {
        super(parent, "vision", "blips", StructBuffer.create(Blip.struct));
        LoggerFactory log = parent.type(this);
        LoggerFactory calLog = log.name("calibration");
        m_log_cameraToTag_factory = calLog.name("camera to tag");
        m_log_robotToTag_factory = calLog.name("robot to tag");
        m_layout = layout;
        m_history = history;
        m_visionUpdater = visionUpdater;
        m_alliance = alliance;
        m_allTags = new TrailingHistory<>();
        m_usedTags = new TrailingHistory<>();
        m_log_cameraToTag = new HashMap<>();
        m_log_tagInRobot = new HashMap<>();

        m_log_allTags = fieldLogger.doubleArrayLogger(Level.TRACE, "all tags");
        m_log_usedTags = fieldLogger.doubleArrayLogger(Level.TRACE, "used tags");

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_pub_used_tags = inst.getStructArrayTopic("used tags", Pose3d.struct).publish();
        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();

        m_log_alliance = log.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = log.doubleLogger(Level.TRACE, "heed radius");
        m_log_tag_error = log.doubleLogger(Level.TRACE, "tag error");
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");

        // Default heed radius is 3.5 meters.
        setHeedRadiusM(3.5);
    }

    /**
     * Clean the history, relative to the current moment.
     * 
     * Previously, eviction only occurred when the robot could see something.
     */
    @Override
    protected void beginUpdate() {
        double deadline = Takt.get() - HISTORY_DURATION;
        m_usedTags.evict(deadline);
        m_allTags.evict(deadline);
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     */
    @Override
    protected void perValue(
            Camera camera,
            Blip[] blips) {

        Transform3d cameraOffset = Offset.get(camera).offset();

        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            if (DEBUG)
                System.out.println("no alliance!");
            return;
        }
        Alliance alliance = optAlliance.get();
        m_log_alliance.log(() -> alliance);

        // Sample the history.

        if (blips.length == 0) {
            if (DEBUG)
                System.out.println("no blips!");
        }

        for (int i = 0; i < blips.length; ++i) {
            Blip blip = blips[i];

            // Camera-to-tag.
            final Transform3d cameraToTag = tagInCamera(blip);

            if (i == 0) {
                // Only log the first tag seen; for calibration we really only see one.
                logCalibration(camera, cameraToTag);
            }

            double timeSec = (double) blip.getTimestamp() / 1e6;
            m_log_lag.log(() -> Takt.get() - timeSec);
            Pose2d samplePose = sample(timeSec);

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldOpt = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldOpt.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag.
            // This is not an estimate, it's the canonical pose from JSON.
            final Pose3d tagInField = tagInFieldOpt.get();

            // Do not override tag rotation
            // tagInCamera = maybeOverrideRotation(cameraOffset, samplePose, tagInField,
            // tagInCamera);

            // Estimate the tag pose in the field frame.
            Pose3d estimatedTagInField = estimatedTagInField(cameraOffset, samplePose, cameraToTag);
            m_allTags.add(timeSec, estimatedTagInField);
            logTagError(tagInField, estimatedTagInField);

            // Compute the pose implied by the vision input.
            Pose2d robotPose2d = robotPose2d(samplePose, cameraOffset, tagInField, cameraToTag);
            if (DEBUG)
                System.out.printf("robotPose2d %s\n", robotPose2d);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (!Experiments.instance.enabled(Experiment.HeedVision)) {
                if (DEBUG)
                    System.out.println("Drop update, vision is off.");
                continue;
            }
            ///
            if (cameraToTag.getTranslation().getNorm() > m_heedRadiusM) {
                if (DEBUG)
                    System.out.println("Tag is too far away.");
                continue;
            }
            ///
            if (m_prevPose == null) {
                // No, we need another nearby fix to believe either one.
                m_prevPose = robotPose2d;
                if (DEBUG)
                    System.out.println("Need confirmation.");
                continue;
            }
            ///
            if (Metrics.translationalDistance(m_prevPose, robotPose2d) > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = robotPose2d;
                if (DEBUG)
                    System.out.println("New estimate is too far away.");
                continue;
            }
            ///
            /// Yes, we should use this update.
            ///
            //////////////////////////////////////////////////////////////////

            m_usedTags.add(timeSec, estimatedTagInField);

            NoisyPose2d noisyMeasurement = new NoisyPose2d(
                    robotPose2d,
                    VisionNoise.get(
                            cameraToTag.getTranslation().getNorm(),
                            Metrics.offAxisAngleRad(cameraToTag)));

            m_visionUpdater.put(timeSec, noisyMeasurement);
            m_prevPose = robotPose2d;
        }

    }

    @Override
    protected void finishUpdate() {
        m_pub_tags.set(m_allTags.getAll().toArray(new Pose3d[0]));
        m_pub_used_tags.set(m_usedTags.getAll().toArray(new Pose3d[0]));
        m_log_allTags.log(
                () -> m_allTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), x.toPose2d().getRotation().getDegrees())).toArray());
        m_log_usedTags.log(
                () -> m_usedTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), x.toPose2d().getRotation().getDegrees())).toArray());
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
        m_log_heedRadius.log(() -> m_heedRadiusM);
    }

    void logCalibration(Camera camera, Transform3d cameraToTag) {
        Transform3dLogger logCameraToTag = m_log_cameraToTag.computeIfAbsent(
                camera.name(),
                (x) -> m_log_cameraToTag_factory.transform3dLogger(Level.TRACE, x));
        logCameraToTag.log(() -> cameraToTag);
        // when correctly calibreated, this should match the actual robot-to-tag
        Transform3dLogger logRobotToTag = m_log_tagInRobot.computeIfAbsent(
                camera.name(),
                (x) -> m_log_robotToTag_factory.transform3dLogger(Level.TRACE, x));
        Transform3d robotToTag = Offset.get(camera).offset().plus(cameraToTag);
        logRobotToTag.log(() -> robotToTag);
    }

    /**
     * Compute the robot pose implied by the vision input.
     * 
     * @param historicalPose sampled from history.
     * @param cameraInRobot  camera offset, from Camera.java.
     * @param tagInField     tag pose from JSON.
     * @param tagInCamera    tag transform in camera frame.
     */
    private Pose2d robotPose2d(
            Pose2d historicalPose,
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera) {
        // Robot in field frame, just using the camera.
        Pose3d robotPose3d = PoseEstimationHelper.robotInField(
                cameraInRobot, tagInField, tagInCamera);
        Pose2d robotPose2d = robotPose3d.toPose2d();
        // we used to override the rotation
        // Pose2d robotPose2d = new Pose2d(
        // robotPose3d.getTranslation().toTranslation2d(),
        // historicalPose.getRotation());
        m_log_pose.log(() -> robotPose2d);
        m_pub_pose.set(robotPose2d);
        return robotPose2d;
    }

    /** Log the norm of the translational error of the tag. */
    private void logTagError(Pose3d tagInField, Pose3d estimatedTagInField) {
        Transform3d tagError = tagInField.minus(estimatedTagInField);
        m_log_tag_error.log(() -> tagError.getTranslation().getNorm());
    }

    /**
     * Sample the history at the frame timestamp.
     */
    private Pose2d sample(double timestamp) {
        // Note this pulls from the *old history*, not the *odometry-updated history*,
        // because we don't care about the latest odometry update.
        //
        // Because the camera delay is much more than the odometry delay, we're always
        // trying to write history from several cycles ago (followed by replay). It's ok
        // for new odometry to be the last thing.
        Pose2d historicalPose = m_history.apply(timestamp).pose();
        if (DEBUG) {
            System.out.printf("historical pose rotation %f\n",
                    historicalPose.getRotation().getRadians());
        }
        return historicalPose;
    }

    /**
     * Use the pose sample, camera offset, and tag-in-camera transform to estimate
     * the tag pose in the field frame.
     */
    private Pose3d estimatedTagInField(
            Transform3d cameraOffset, Pose2d historicalPose, Transform3d tagInCamera) {
        // Field-to-robot
        Pose3d historicalPose3d = new Pose3d(historicalPose);
        // Field-to-robot plus robot-to-camera = field-to-camera
        Pose3d historicalCameraInField = historicalPose3d.transformBy(cameraOffset);
        // Given the historical pose, where do we think the tag is?
        return historicalCameraInField.transformBy(tagInCamera);
    }

    /**
     * Camera-to-tag, as it appears in the camera frame.
     * The raw pose in the blip is "z-forward" like the camera.
     * This returns "x-forward" like the robot.
     */
    private static Transform3d tagInCamera(Blip blip) {
        return blip.blipToTransform();
    }

}