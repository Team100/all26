package org.team100.frc2026.robot;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.subsystems.Conveyor;
import org.team100.frc2026.subsystems.Feeder;
import org.team100.frc2026.subsystems.Intake;
import org.team100.frc2026.subsystems.IntakeExtend;
import org.team100.frc2026.subsystems.Shooter;
import org.team100.frc2026.targeting.Targeter;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.NoEstimate;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.localization.StateEstimatorProxy;
import org.team100.lib.localization.fused.AprilTagCornerReader;
import org.team100.lib.localization.fused.FusedEstimator;
import org.team100.lib.localization.fused.NudgingVisionEstimator;
import org.team100.lib.localization.fused.OdometryEstimator;
import org.team100.lib.localization.fused.UpdatedEstimate;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.simulation.GroundTruth;
import org.team100.lib.localization.visualization.AprilTagVisualizer;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.music.Player;
import org.team100.lib.music.Polyphony;
import org.team100.lib.music.Unison;
import org.team100.lib.music.commands.Play;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.state.StateSE2;
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
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.SwerveHistoryVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery implements AutoCloseable {
    private final SwerveModuleCollection m_modules;
    private final GroundTruth m_groundTruth;
    private final StateEstimator m_proxyEstimator;

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
    public final Feeder m_feeder;
    public final Conveyor m_conveyor;

    public Machinery(LoggerFactory logger, LoggerFactory fieldLogger, TotalCurrentLog currentLog) {
        LoggerFactory driveLog = logger.name("Drive");

        ////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_modules = SwerveModuleCollection.get(
                driveLog,
                currentLog,
                new CurrentLimit(67.5, 52.5),
                new CurrentLimit(45, 22.5));
        Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        AprilTagFieldLayoutWithCorrectOrientation layout = AprilTagFieldLayoutWithCorrectOrientation.getLayout();
        SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);
        AprilTagCornerReader tagReader = new AprilTagCornerReader(
                driveLog, layout, MatchState::getAlliance);
        OdometryEstimator odometryEstimate = new OdometryEstimator(
                driveLog,
                m_swerveKinodynamics,
                gyro.white_noise(),
                gyro.bias_noise(),
                RobotBase.isSimulation(),
                false);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(driveLog);
        FusedEstimator fusedEstimate = new FusedEstimator(
                Takt.get(),
                gyro::getYawNWU,
                swerveLocal::positions,
                tagReader::read,
                odometryEstimate,
                visionEstimate);
        m_proxyEstimator = new StateEstimatorProxy(
                new UpdatedEstimate(fusedEstimate), new NoEstimate());
        m_drive = new SwerveDriveSubsystem(
                driveLog,
                m_proxyEstimator,
                swerveLocal);
        new AprilTagVisualizer(
                driveLog, fieldLogger,
                t -> m_drive.getState(t).state(), layout, MatchState::getAlliance);
        new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState(), "robot");
        new SwerveHistoryVisualization(fieldLogger, m_drive);

        ////////////////////////////////
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

        m_targets = new Targets(driveLog, fieldLogger, 0.2, t -> m_drive.getState(t).state());

        /////////////////////////////////
        //
        // SUBSYSTEMS
        //
        m_intake = new Intake(logger, currentLog);
        m_intakeExtend = new IntakeExtend(logger, currentLog);
        m_shooter = new Shooter(logger, currentLog, m_cachedSolution::speed);
        m_feeder = new Feeder(logger, currentLog, m_shooter);
        m_conveyor = new Conveyor(logger, currentLog);

        /////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);

        /////////////////////////////////
        //
        // INDICATOR
        //
        // Beeper makes beeps to warn about testing.
        m_beeper = new Beeper(m_drive);

        ////////////////////////////////////////////////////////////
        ///
        /// GROUND TRUTH
        ///
        m_groundTruth = new GroundTruth(
                fieldLogger, logger, m_swerveKinodynamics, m_modules, layout);

        ///////////////////////////////////
        /// 
        /// EXPERIMENT OVERRIDES
        /// 

        // Without cameras, this should always be on.
        // Experiments.INSTANCE.override(Experiment.PerfectGyro, true);
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
        StateSE2 state = new StateSE2(p.pose(), VelocitySE2.ZERO);
        m_drive.reset(Takt.get(), new NoisyStateSE2(state, p.noise()));
        // also reset the ground truth, otherwise the cameras retain the old pose
        m_groundTruth.resetPose(state);
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

    public Command play() {
        // six
        Unison voice1 = new Unison(
                Stream.concat(
                        m_feeder.players().stream(),
                        m_shooter.players().stream()).toArray(Player[]::new));
        // six
        Unison voice2 = new Unison(
                Stream.concat(
                        Stream.concat(
                                m_conveyor.players().stream(),
                                m_intake.players().stream()),
                        m_intakeExtend.players().stream()).toArray(Player[]::new));
        // eight
        Unison voice3 = new Unison(
                m_drive.players().toArray(Player[]::new));
        Polyphony allVoices = new Polyphony(voice1, voice2, voice3);
        return new Play("jupiter.mid", 1.0, allVoices);
    }

    /**
     * Keeps the tests from conflicting via the use of simulated HAL ports.
     */
    @Override
    public void close() {
        m_modules.close();
        m_solver.close();
        m_proxyEstimator.close();
    }

}
