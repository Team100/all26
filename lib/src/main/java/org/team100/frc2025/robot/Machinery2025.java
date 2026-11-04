package org.team100.frc2025.robot;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.Climber.Climber2025;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.grip.Manipulator;
import org.team100.frc2025.indicator.LEDIndicator;
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
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.targeting.Targets;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */
public class Machinery2025 {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;
    private final LoggerFactory logger;
    private final LoggerFactory fieldLogger;

    private final SwerveModuleCollection m_modules;
    private final LEDIndicator m_leds;
    private final GroundTruth m_groundTruth;

    final CalgamesMech m_mech;
    final Manipulator m_manipulator;
    final Climber2025 m_climber;
    final ClimberIntake m_climberIntake;
    final TrajectoryVisualization m_trajectoryViz;
    final SwerveKinodynamics m_swerveKinodynamics;
    final Targets m_targets;
    final SwerveDriveSubsystem m_drive;
    final Beeper m_beeper;

    public Machinery2025(LoggerFactory root, LoggerFactory field, TotalCurrentLog currentLog) {
        logger = root;
        fieldLogger = field;
        LoggerFactory driveLog = logger.name("Drive");

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_modules = SwerveModuleCollection.get(
                driveLog,
                currentLog,
                new CurrentLimit(DRIVE_STATOR_LIMIT, DRIVE_SUPPLY_LIMIT),
                new CurrentLimit(30, 20));
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
                driveLog, layout, DriverStation::getAlliance);
        OdometryEstimator odometryEstimate = new OdometryEstimator(
                logger,
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
        StateEstimator proxyEstimator = new StateEstimatorProxy(
                new UpdatedEstimate(fusedEstimate), new NoEstimate());
        m_drive = new SwerveDriveSubsystem(
                driveLog,
                proxyEstimator,
                swerveLocal);
        new AprilTagVisualizer(
                driveLog, fieldLogger,
                t -> m_drive.getState(t).state(),
                layout, DriverStation::getAlliance);
        new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState(), "robot");

        ////////////////////////////////////////////////////////////
        //
        // TARGETING
        //
        m_targets = new Targets(
                driveLog, fieldLogger, 0.2, t -> m_drive.getState(t).state());

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //
        m_mech = new CalgamesMech(logger, currentLog, 0.5, 0.343);
        m_manipulator = new Manipulator(logger, currentLog);
        m_climber = new Climber2025(logger, currentLog, new CanId(13));
        m_climberIntake = new ClimberIntake(logger, currentLog, new CanId(14));

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);
        new CalgamesViz(m_mech);
        new ClimberVisualization(m_climber, m_climberIntake);

        ////////////////////////////////////////////////////////////
        //
        // LED INDICATOR
        //
        m_leds = new LEDIndicator(m_manipulator, m_climberIntake);
        m_beeper = new Beeper(m_mech, m_manipulator, m_drive);

        ////////////////////////////////////////////////////////////
        ///
        /// GROUND TRUTH
        ///
        m_groundTruth = new GroundTruth(fieldLogger, logger, m_swerveKinodynamics, m_modules, layout);
    }

    public void resetPose(NoisyPose2d p) {
        StateSE2 state = new StateSE2(p.pose(), VelocitySE2.ZERO);
        m_drive.reset(Takt.get(), new NoisyStateSE2(state, p.noise()));
        // also reset the ground truth, otherwise the cameras retain the old pose
        m_groundTruth.resetPose(state);
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
        m_leds.close();
    }

}
