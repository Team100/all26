package org.team100.frc2026.robot;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.subsystems.Intake;
import org.team100.frc2026.subsystems.IntakeExtend;
import org.team100.frc2026.subsystems.Shooter;
import org.team100.frc2026.targeting.Targeter;
import org.team100.lib.coherence.Takt;
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
import org.team100.lib.targeting.CachedSolution;
import org.team100.lib.targeting.ProxySolver;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
    // 3/14/26 lowered from 90 to 80

    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final Runnable m_robotViz;
    private final SwerveModuleCollection m_modules;
    private final GroundTruth m_groundTruth;

    public final TrajectoryVisualization m_trajectoryViz;
    public final SwerveKinodynamics m_swerveKinodynamics;
    public final NudgingVisionUpdater m_visionUpdater;
    public final AprilTagRobotLocalizer m_localizer;
    public final SwerveLimiter m_limiter;
    public final SwerveDriveSubsystem m_drive;
    public final Beeper m_beeper;

    public final ProxySolver m_solver;
    public final CachedSolution m_cachedSolution;

    public final Shooter m_shooter;
    public final Intake m_intake;
    public final IntakeExtend m_intakeExtend;
    public final ControllerSE2 m_holonomicController;

    public Machinery(TotalCurrentLog currentLog) {

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
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();

        m_modules = SwerveModuleCollection.get(
                driveLog,
                currentLog,
                CurrentLimits.DRIVE,
                CurrentLimits.STEERING,
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
        m_visionUpdater = new NudgingVisionUpdater(
                driveLog, history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.getLayout();
        m_localizer = new AprilTagRobotLocalizer(
                driveLog,
                fieldLogger,
                layout,
                history,
                m_visionUpdater,
                DriverStation::getAlliance);

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
        // TARGETING
        //

        Targeter targeter = new Targeter(() -> m_drive.getState().translation());
        m_solver = new ProxySolver(targeter::forRange);

        Supplier<Optional<Translation2d>> target = () -> {
            return FieldConstants2026.TARGET(
                    m_drive.getState().translation());
        };

        m_cachedSolution = new CachedSolution(
                fieldLogger, m_drive::getState, target, m_solver);

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //

        m_intake = new Intake(logger, currentLog);
        m_intakeExtend = new IntakeExtend(logger, currentLog);
        m_shooter = new Shooter(logger, currentLog, m_cachedSolution::speed);

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

    /**
     * Nudge the rotation towards zero, like a camera would do.
     * The "nudge" in this case is quite firm.
     */
    public Command zeroRotation() {
        return Commands.runOnce(() -> {
            Pose2d p = m_drive.getPose();
            NoisyPose2d np = new NoisyPose2d(
                    new Pose2d(p.getX(), p.getY(), Rotation2d.kZero),
                    IsotropicNoiseSE2.fromStdDev(10, 0.001));
            System.out.printf("*** ZERO ROTATION: %s\n", np);
            m_visionUpdater.put(Takt.get(), np);
        }, m_drive);
    }

    public void periodic() {
        m_groundTruth.periodic();
        // publish pose estimate
        m_robotViz.run();
    }

    /**
     * Keeps the tests from conflicting via the use of simulated HAL ports.
     */
    public void close() {
        m_modules.close();
        m_solver.close();
    }

}
