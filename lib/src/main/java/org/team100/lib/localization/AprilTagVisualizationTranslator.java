package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.AprilTagVisualizer.Measurement;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.StateSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Convert an array of BlipsWithCorners into a list of
 * AprilTagVisualizer.Measurement
 */
public class AprilTagVisualizationTranslator {
    private static final boolean DEBUG = false;

    private final DoubleFunction<StateSE2> m_history;
    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final PoseFromCorners m_estimator;
    private final DoubleLogger m_log_tag_error;

    public AprilTagVisualizationTranslator(
            LoggerFactory parent,
            DoubleFunction<StateSE2> history,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        m_history = history;
        m_alliance = alliance;
        m_layout = layout;
        m_estimator = new PoseFromCorners();
        m_log_tag_error = log.doubleLogger(Level.DEBUG, "tag error");

    }

    List<Measurement> convert(Camera camera, BlipWithCorners[] blips) {
        if (!Experiments.INSTANCE.enabled(Experiment.ShowTags))
            return List.of();
        Transform3d cameraOffset = Offset.get(camera).offset();
        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            if (DEBUG)
                System.out.println("no alliance!");
            return List.of();
        }
        Alliance alliance = optAlliance.get();

        List<Measurement> measurements = new ArrayList<>();

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

            // Log the norm of the translational error of the tag.
            m_log_tag_error.log(
                    () -> tagInField.minus(estimatedTagInField).getTranslation().getNorm());

            measurements.add(new Measurement(blipTimeSec, estimatedTagInField));
        }
        return measurements;
    }

    /** Sample the history at the frame timestamp. */
    private Pose2d sample(double timestamp) {
        // Note this pulls from the *old history*, not the *odometry-updated history*,
        // because we don't care about the latest odometry update.
        //
        // Because the camera delay is much more than the odometry delay, we're always
        // trying to write history from several cycles ago (followed by replay). It's ok
        // for new odometry to be the last thing.
        return m_history.apply(timestamp).pose();
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
