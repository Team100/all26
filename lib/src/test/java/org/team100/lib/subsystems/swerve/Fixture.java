package org.team100.lib.subsystems.swerve;

import java.io.IOException;
import java.util.function.UnaryOperator;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.controller.se2.ControllerFactorySE2;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 * 
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class Fixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveHistory history;
    public FreshSwerveEstimate estimate;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public OdometryUpdater odometryUpdater;
    public SwerveLimiter limiter;
    public SwerveDriveSubsystem drive;
    public ControllerSE2 controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public Fixture() throws IOException {
        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20), swerveKinodynamics);
        gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        history = new SwerveHistory(
                logger,
                swerveKinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                SwerveModulePositions.kZero(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // initial time is zero here for testing
        // history.reset(gyro.getYawNWU(), collection.positions(), Pose2d.kZero, 0);

        odometryUpdater = new OdometryUpdater(
                logger, swerveKinodynamics, gyro, history, collection::positions, UnaryOperator.identity());
        odometryUpdater.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);

        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                logger, history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                logger, fieldLogger, layout, history, visionUpdater,DriverStation::getAlliance);
        estimate = new FreshSwerveEstimate(
                localizer::update, odometryUpdater::update, history);

        limiter = new SwerveLimiter(logger, swerveKinodynamics, () -> 12);
        drive = new SwerveDriveSubsystem(
                logger,
                odometryUpdater,
                estimate,
                swerveLocal);

        controller = ControllerFactorySE2.test(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
