package org.team100.lib.localization.fused;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.localization.PoseEstimationHelper;
import org.team100.lib.localization.PoseFromCorners;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VisionNoise;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Translates a list of camera observations into a list of robot poses.
 */
public class AprilTagCornerTranslator {
    private static final boolean DEBUG = false;

    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final PoseFromCorners m_estimator;

    private final LoggerFactory m_log_cameraToTag_factory;
    private final LoggerFactory m_log_robotToTag_factory;
    private final Map<String, Transform3dLogger> m_log_cameraToTag;
    private final Map<String, Transform3dLogger> m_log_tagInRobot;

    public AprilTagCornerTranslator(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        LoggerFactory calLog = log.name("calibration");
        m_layout = layout;
        m_alliance = alliance;
        m_estimator = new PoseFromCorners();
        m_log_cameraToTag_factory = calLog.name("camera to tag");
        m_log_robotToTag_factory = calLog.name("robot to tag");
        m_log_cameraToTag = new HashMap<>();
        m_log_tagInRobot = new HashMap<>();
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     */
    public List<VisionMeasurement> convert(Camera camera, BlipWithCorners[] blips) {
        Transform3d cameraOffset = Offset.get(camera).offset();

        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            if (DEBUG)
                System.out.println("no alliance!");
            return List.of();
        }
        Alliance alliance = optAlliance.get();

        if (blips.length == 0) {
            if (DEBUG)
                System.out.println("no blips!");
        }

        List<VisionMeasurement> measurements = new ArrayList<>();
        for (int i = 0; i < blips.length; ++i) {
            BlipWithCorners blip = blips[i];

            // Camera-to-tag.
            Transform3d cameraToTag = tagInCamera(camera, blip);

            if (i == 0) {
                // Only log the first tag seen; for calibration we really only see one.
                logCalibration(camera, cameraToTag);
            }

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldOpt = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldOpt.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from JSON map.
            Pose3d tagInField = tagInFieldOpt.get();

            // Compute the pose implied by the vision input.
            Pose2d robotPose2d = robotPose2d(cameraOffset, tagInField, cameraToTag);

            if (DEBUG)
                System.out.printf("robotPose2d %s\n", robotPose2d);

            // Estimate the tag pose in the field frame.
            double blipTimeSec = (double) blip.getTimestamp() / 1e6;

            NoisyPose2d noisyMeasurement = new NoisyPose2d(
                    robotPose2d,
                    VisionNoise.get(
                            cameraToTag.getTranslation().getNorm(),
                            Metrics.offAxisAngleRad(cameraToTag)));

            measurements.add(new VisionMeasurement(blipTimeSec, noisyMeasurement));
        }
        return measurements;
    }

    /**
     * Compute the robot pose implied by the vision input.
     * 
     * @param cameraInRobot camera offset, from Camera.java.
     * @param tagInField    tag pose from JSON.
     * @param tagInCamera   tag transform in camera frame.
     */
    private Pose2d robotPose2d(
            Transform3d cameraInRobot,
            Pose3d tagInField,
            Transform3d tagInCamera) {
        // Robot in field frame, just using the camera.
        Pose3d robotPose3d = PoseEstimationHelper.robotInField(
                cameraInRobot, tagInField, tagInCamera);
        return robotPose3d.toPose2d();
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

    void logCalibration(Camera camera, Transform3d cameraToTag) {
        Transform3dLogger logCameraToTag = m_log_cameraToTag.computeIfAbsent(
                camera.name(),
                (x) -> m_log_cameraToTag_factory.transform3dLogger(Level.TRACE, x));
        logCameraToTag.log(() -> cameraToTag);
        // when correctly calibrated, this should match the actual robot-to-tag
        Transform3dLogger logRobotToTag = m_log_tagInRobot.computeIfAbsent(
                camera.name(),
                (x) -> m_log_robotToTag_factory.transform3dLogger(Level.TRACE, x));
        Transform3d robotToTag = Offset.get(camera).offset().plus(cameraToTag);
        logRobotToTag.log(() -> robotToTag);
    }
}
