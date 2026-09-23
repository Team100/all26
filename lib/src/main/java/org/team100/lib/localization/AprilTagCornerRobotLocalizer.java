package org.team100.lib.localization;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.logging.LoggerFactory.Transform3dLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VisionNoise;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.util.struct.StructBuffer;



/**
 * Uses the corners of observed AprilTags to derive the robot pose.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagCornerRobotLocalizer extends CameraReader<BlipWithCorners> {
    private static final boolean DEBUG = false;
    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;
    private final PoseFromCorners m_estimator;
    private final VisionUpdater m_visionUpdater;
    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final LoggerFactory m_log_cameraToTag_factory;
    private final LoggerFactory m_log_robotToTag_factory;
    private final DoubleLogger m_log_heedRadius;
    private final Pose2dLogger m_log_pose;
    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final DoubleLogger m_log_lag;
    private final Map<String, Transform3dLogger> m_log_cameraToTag;
    private final Map<String, Transform3dLogger> m_log_tagInRobot;
    /**
     * Remember the previous vision-based pose estimate, so we can measure the
     * distance between consecutive updates, and ignore too-far updates.
     */
    private Pose2d m_prevPose;
    /** Use tags closer than this. Ignore tags further than this. */
    private double m_heedRadiusM;

    public AprilTagCornerRobotLocalizer(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            VisionUpdater visionUpdater,
            Supplier<Optional<Alliance>> alliance) {
        super(parent, "vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
        LoggerFactory log = parent.type(this);
        LoggerFactory calLog = log.name("calibration");
        m_log_cameraToTag_factory = calLog.name("camera to tag");
        m_log_robotToTag_factory = calLog.name("robot to tag");
        m_estimator = new PoseFromCorners();
        m_layout = layout;
        m_visionUpdater = visionUpdater;
        m_alliance = alliance;
        m_log_cameraToTag = new HashMap<>();
        m_log_tagInRobot = new HashMap<>();
        m_log_heedRadius = log.doubleLogger(Level.TRACE, "heed radius");
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");
        setHeedRadiusM(3.5);
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     */
    @Override
    protected void perValue(Camera camera, BlipWithCorners[] blips) {
        Transform3d cameraOffset = Offset.get(camera).offset();

        // Fetch the alliance (not available immediately after startup).
        Optional<Alliance> optAlliance = m_alliance.get();
        if (!optAlliance.isPresent()) {
            if (DEBUG)
                System.out.println("no alliance!");
            return;
        }
        Alliance alliance = optAlliance.get();

        if (blips.length == 0) {
            if (DEBUG)
                System.out.println("no blips!");
        }

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
            // log every single update. maybe remove this?
            m_log_pose.log(() -> robotPose2d);
            if (DEBUG)
                System.out.printf("robotPose2d %s\n", robotPose2d);

            // Estimate the tag pose in the field frame.
            double blipTimeSec = (double) blip.getTimestamp() / 1e6;
            m_log_lag.log(() -> Takt.get() - blipTimeSec);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
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
            double distanceFromPrev = Metrics.translationalDistance(m_prevPose, robotPose2d);
            if (distanceFromPrev > VISION_CHANGE_TOLERANCE_M) {
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

            NoisyPose2d noisyMeasurement = new NoisyPose2d(
                    robotPose2d,
                    VisionNoise.get(
                            cameraToTag.getTranslation().getNorm(),
                            Metrics.offAxisAngleRad(cameraToTag)));

            m_visionUpdater.put(blipTimeSec, noisyMeasurement);
            m_prevPose = robotPose2d;
        }
    }

    /**
     * Tags outside this radius are ignored.
     */
    void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
        m_log_heedRadius.log(() -> m_heedRadiusM);
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

}