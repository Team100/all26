package org.team100.frc2025.robot;

import java.util.function.UnaryOperator;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.Climber.Climber2025;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.grip.Manipulator;
import org.team100.frc2025.indicator.LEDIndicator;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.indicator.Beeper;
import org.team100.lib.localization.AddOdometryNoise;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagVisualizer;
import org.team100.lib.localization.FusedEstimator;
import org.team100.lib.localization.GroundTruth;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.GyroFactory;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.targeting.Targets;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.RobotPoseVisualization;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Twist2d;
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
    private static final LoggerFactory logger = Logging.instance().rootLogger;
    private static final LoggerFactory fieldLogger = Logging.instance().fieldLogger;

    private final RobotPoseVisualization m_robotViz;
    private final AprilTagVisualizer m_tagViz;
    private final Runnable m_combinedViz;
    private final Runnable m_climberViz;
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

    public Machinery2025(TotalCurrentLog currentLog) {
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
                driveLog, fieldLogger, m_drive::getState, layout, DriverStation::getAlliance);
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_drive.getState().pose(), "robot");

        ////////////////////////////////////////////////////////////
        //
        // TARGETING
        //
        m_targets = new Targets(driveLog, fieldLogger, 0.2, (t) -> m_drive.getState(t));

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
        m_combinedViz = new CalgamesViz(m_mech);
        m_climberViz = new ClimberVisualization(m_climber, m_climberIntake);

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

    public void periodic() {
        m_groundTruth.periodic();
        m_leds.periodic();
        m_robotViz.run();
        m_combinedViz.run();
        m_climberViz.run();
        m_tagViz.update();
    }

    public void close() {
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
        m_leds.close();
    }

}
