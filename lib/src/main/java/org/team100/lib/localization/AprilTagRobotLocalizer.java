package org.team100.lib.localization;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
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
 * Extracts robot pose estimates from camera observations of AprilTags.
 * 
 * Note this class depends only on the state *history*, not on the coherent sate
 * *estimate*. The camera input doesn't require fresh odometry, it modifies the
 * past (and replays up to the present).
 */
public class AprilTagRobotLocalizer {
    private static final boolean DEBUG = false;

    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;
    private final CameraReader<Blip> m_reader;
    private final VisionUpdater m_visionUpdater;
    private final Supplier<Optional<Alliance>> m_alliance;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;

    // LOGGERS
    private final LoggerFactory m_log_cameraToTag_factory;
    private final LoggerFactory m_log_robotToTag_factory;

    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;
    private final Pose2dLogger m_log_pose;

    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

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
     * @param history       f(timestamp) = swerve state
     * @param visionUpdater mutates history
     */
    public AprilTagRobotLocalizer(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            VisionUpdater visionUpdater,
            Supplier<Optional<Alliance>> alliance) {
        m_reader = new CameraReader<>("vision", "blips",
                StructBuffer.create(Blip.struct));
        LoggerFactory log = parent.type(this);
        LoggerFactory calLog = log.name("calibration");
        m_log_cameraToTag_factory = calLog.name("camera to tag");
        m_log_robotToTag_factory = calLog.name("robot to tag");
        m_layout = layout;
        m_visionUpdater = visionUpdater;
        m_alliance = alliance;
        m_log_cameraToTag = new HashMap<>();
        m_log_tagInRobot = new HashMap<>();
        m_log_alliance = log.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = log.doubleLogger(Level.TRACE, "heed radius");
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");
        // Default heed radius is 3.5 meters.
        setHeedRadiusM(3.5);
    }

    public void update() {
        List<CameraReader.Record<Blip>> records = m_reader.getRecords();
        for (CameraReader.Record<Blip> r : records) {
            perValue(r.camera(), r.values());
        }
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     */
    protected void perValue(Camera camera, Blip[] blips) {

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

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldOpt = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldOpt.isPresent()) {
                // This shouldn't happen, but it does.
                System.out.printf("WARNING: VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from JSON map.
            final Pose3d tagInField = tagInFieldOpt.get();

            // Compute the pose implied by the vision input.
            Pose2d robotPose2d = robotPose2d(cameraOffset, tagInField, cameraToTag);
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
            if (Metrics.translationalDistance(m_prevPose, robotPose2d) > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = robotPose2d;
                if (DEBUG)
                    System.out.printf("New estimate %s is too far away from old %s.", robotPose2d, m_prevPose);
                continue;
            }
            ///
            /// Yes, we should use this update.
            ///
            //////////////////////////////////////////////////////////////////

            if (DEBUG)
                System.out.printf("add pose %s\n", robotPose2d);

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
        Pose2d robotPose2d = robotPose3d.toPose2d();
        m_log_pose.log(() -> robotPose2d);
        return robotPose2d;
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