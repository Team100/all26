package frc.robot;

import java.io.IOException;
import java.util.function.UnaryOperator;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.AddOdometryNoise;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.GroundTruth;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.system.RobotController;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final CurrentLimit DRIVE_LIMIT = new CurrentLimit(110, 90);
    private static final CurrentLimit STEER_LIMIT = new CurrentLimit(40, 40);

    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Runnable m_robotViz;
    private final SwerveModuleCollection m_modules;
    private final GroundTruth m_groundTruth;

    public final TrajectoryVisualization m_trajectoryViz;
    public final SwerveKinodynamics m_swerveKinodynamics;
    public final AprilTagRobotLocalizer m_localizer;
    public final SwerveLimiter m_limiter;
    public final SwerveDriveSubsystem m_drive;
    public final Beeper m_beeper;
    public final Climber m_Climber;

    public final ClimberExtension m_ClimberExtension;

    public final ControllerSE2 m_holonomicController;

    public Machinery() {

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);

        ////////////////////////////////////////////////////////////
        //
        // POSE ESTIMATION
        //
        LoggerFactory driveLog = logger.name("Drive");
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();

        m_modules = SwerveModuleCollection.get(
                driveLog,
                currentLog,
                DRIVE_LIMIT,
                STEER_LIMIT,
                m_swerveKinodynamics);
        Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        SwerveHistory history = new SwerveHistory(
                driveLog,
                m_swerveKinodynamics,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                m_modules.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        UnaryOperator<Twist2d> odometryNoise = RobotBase.isReal() ? UnaryOperator.identity() : new AddOdometryNoise();
        OdometryUpdater odometryUpdater = new OdometryUpdater(
                driveLog,
                m_swerveKinodynamics,
                gyro,
                history,
                m_modules::positions,
                odometryNoise);
        // odometryUpdater.m_debug = true;
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high());
        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                driveLog, history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        AprilTagFieldLayoutWithCorrectOrientation layout = getLayout();
        m_localizer = new AprilTagRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                visionUpdater,
                MatchState::getAlliance);

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_limiter = new SwerveLimiter(
                driveLog,
                m_swerveKinodynamics,
                RobotController::getBatteryVoltage);
        m_drive = SwerveDriveFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_localizer,
                odometryUpdater,
                history,
                m_modules);
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState().pose(), "robot");

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //

        m_ClimberExtension = new ClimberExtension(logger, currentLog);
        m_Climber = new Climber(logger, currentLog);
        ////////////////////////////////////////////////////////////
        ///
        /// GROUND TRUTH
        ///
        m_groundTruth = new GroundTruth(fieldLogger, logger, m_swerveKinodynamics, m_modules, layout);

        ////////////////////////////////////////////////////////////
        //
        // INDICATOR
        //
        // There's no LED this year, unless we need it for testing or setup.
        // Beeper makes beeps to warn about testing.
        m_beeper = new Beeper(m_drive);

        ////////////////////////////////////////////////////////////
        //
        // CONTROLLER
        //
        m_holonomicController = ControllerFactorySE2.byIdentity(driveLog);
    }

    /**
     * Purge the history and assert the given pose as the current estimate, with
     * high variance, so that the robot immediately listens to the cameras to get a
     * new pose.
     */
    public void resetPose(Pose2d p) {
        m_drive.resetPose(p, IsotropicNoiseSE2.high());
        // also reset the ground truth, otherwise the cameras retain the old pose
        m_groundTruth.resetPose(p);
    }

    /** Erase the pose history, use high variance for pose estimate. */
    public Command disorient() {
        return Commands.runOnce(() -> {
            Pose2d p = m_drive.getPose();
            System.out.printf("*** DISORIENT: %s\n", p);
            resetPose(p);
        }, m_drive);
    }

    public void periodic() {
        m_groundTruth.periodic();
        // publish pose estimate
        m_robotViz.run();
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
    }

    /** Trap the IO exception. */
    private static AprilTagFieldLayoutWithCorrectOrientation getLayout() {
        try {
            return new AprilTagFieldLayoutWithCorrectOrientation();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }
}
