package org.team100.frc2026.robot;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.subsystems.Intake;
import org.team100.frc2026.subsystems.IntakeExtend;
import org.team100.frc2026.subsystems.Shooter;
import org.team100.frc2026.targeting.Targeter;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.AddOdometryNoise;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagVisualizer;
import org.team100.lib.localization.FusedEstimator;
import org.team100.lib.localization.GroundTruth;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.targeting.CachedSolution;
import org.team100.lib.targeting.ProxySolver;
import org.team100.lib.targeting.Targets;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery {
    private final RobotPoseVisualization m_robotViz;
    private final AprilTagVisualizer m_tagViz;
    private final SwerveModuleCollection m_modules;
    private final GroundTruth m_groundTruth;

    public final TrajectoryVisualization m_trajectoryViz;
    public final SwerveKinodynamics m_swerveKinodynamics;
    public final SwerveDriveSubsystem m_drive;
    public final Beeper m_beeper;

    public final ProxySolver m_solver;
    public final CachedSolution m_cachedSolution;
    public final Targets m_targets;

    public final Shooter m_shooter;
    public final Intake m_intake;
    public final IntakeExtend m_intakeExtend;

    public Machinery(LoggerFactory logger, LoggerFactory fieldLogger, TotalCurrentLog currentLog) {
        LoggerFactory driveLog = logger.name("Drive");

        //////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_modules = SwerveModuleCollection.get(
                driveLog,
                currentLog,
                CurrentLimits.DRIVE,
                CurrentLimits.STEERING);
        Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.getLayout();
        SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        UnaryOperator<Twist2d> odometryNoise = RobotBase.isReal() ? UnaryOperator.identity() : new AddOdometryNoise();
        FusedEstimator estimate = new FusedEstimator(
                driveLog,
                fieldLogger,
                m_swerveKinodynamics,
                odometryNoise,
                layout,
                gyro,
                swerveLocal);
        m_drive = new SwerveDriveSubsystem(
                driveLog,
                estimate,
                swerveLocal);
        m_tagViz = new AprilTagVisualizer(
                driveLog, fieldLogger, m_drive::getState, layout, MatchState::getAlliance);
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState().pose(), "robot");

        //////////////////////////////////////////////////////////
        //
        // TARGETING
        //

        // Targeting from 2026: aim at the hub, or at the alliance zone.

        Targeter targeter = new Targeter(() -> m_drive.getState().translation());
        m_solver = new ProxySolver(targeter::forRange);

        Supplier<Optional<Translation2d>> target = () -> {
            return FieldConstants2026.TARGET(
                    m_drive.getState().translation());
        };

        m_cachedSolution = new CachedSolution(
                fieldLogger, m_drive::getState, target, m_solver);

        // Targeting from 2025: the cameras are looking for game pieces.

        m_targets = new Targets(driveLog, fieldLogger, 0.2, (t) -> m_drive.getState(t));

        ///////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //
        m_intake = new Intake(logger, currentLog);
        m_intakeExtend = new IntakeExtend(logger, currentLog);
        m_shooter = new Shooter(logger, currentLog, m_cachedSolution::speed);

        ///////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);

        ///////////////////////////////////////////////////////////
        //
        // INDICATOR
        //
        // Beeper makes beeps to warn about testing.
        m_beeper = new Beeper(m_drive);

        ////////////////////////////////////////////////////////////
        ///
        /// GROUND TRUTH
        ///
        m_groundTruth = new GroundTruth(fieldLogger, logger, m_swerveKinodynamics, m_modules, layout);
    }

    /**
     * Purge the history and assert the given pose as the current estimate, with
     * high variance, so that the robot immediately listens to the cameras to get a
     * new pose.
     */
    public void resetPose(Pose2d p) {
        resetPose(new NoisyPose2d(p, IsotropicNoiseSE2.high()));
    }

    /**
     * Purge the history and assert the given pose as the current estimate.
     */
    public void resetPose(NoisyPose2d p) {
        m_drive.resetPose(p.pose(), p.noise());
        // also reset the ground truth, otherwise the cameras retain the old pose
        m_groundTruth.resetPose(p.pose());
    }

    /** Erase the pose history, use high variance for pose estimate. */
    public Command disorient() {
        return Commands.runOnce(() -> {
            Pose2d p = m_drive.getState().pose();
            System.out.printf("*** DISORIENT: %s\n", p);
            resetPose(p);
        }, m_drive);
    }

    /** Force the pose to the origin. FOR TEST ONLY! */
    public Command zeroPose() {
        return Commands.runOnce(() -> {
            Pose2d p = new Pose2d();
            System.out.printf("*** ZERO POSE\n");
            resetPose(p);
        }, m_drive);
    }

    /**
     * Nudge the rotation towards zero, like a camera would do.
     * The "nudge" in this case is quite firm.
     */
    public Command zeroRotation() {
        return Commands.runOnce(() -> {
            Translation2d t = m_drive.getState().pose().getTranslation();
            Pose2d p = new Pose2d(t, Rotation2d.kZero);
            // no influence over cartesian variance
            // a strong claim about rotation variance
            IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(10, 0.001);
            NoisyPose2d np = new NoisyPose2d(p, noise);
            System.out.printf("*** ZERO ROTATION: %s\n", np);
            resetPose(np);
        }, m_drive);
    }

    /** Generally for simulation and visualization */
    public void periodic() {
        m_groundTruth.periodic();
        m_robotViz.run();
        m_tagViz.update();
    }

    /**
     * Keeps the tests from conflicting via the use of simulated HAL ports.
     */
    public void close() {
        m_modules.close();
        m_solver.close();
    }

}
