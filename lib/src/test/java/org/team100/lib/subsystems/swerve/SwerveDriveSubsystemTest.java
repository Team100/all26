package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.FusedEstimator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class SwerveDriveSubsystemTest implements Timeless {

    private static final double DELTA = 0.01;

    @Test
    void test0() throws IOException {

        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        UnaryOperator<Twist2d> odometryNoise = UnaryOperator.identity();

        FusedEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, odometryNoise, layout, gyro, swerveLocal);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);

        Experiments.INSTANCE.override(Experiment.UseSwerveLimiter, false);
        stepTime();

        drive.resetPose(new Pose2d(), IsotropicNoiseSE2.high());

        stepTime();
        StateSE2 state = drive.getState();

        assertEquals(0, state.x().x(), DELTA);
        assertEquals(0, state.x().v(), DELTA);

        // go 1 m/s in +x
        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();

        // at 1 m/s for 0.02 s, so we go 0.02 m
        assertEquals(0.02, collection.positions().frontLeft().distanceMeters(), 1e-6);
        StateSE2 state1 = drive.getState();

        // it took 0.02 s to go from 0 m/s to 1 m/s, so we accelerated 50 m/s/s.
        assertEquals(0.02, state1.x().x(), DELTA);
        assertEquals(1.00, state1.x().v(), DELTA);

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        StateSE2 state2 = drive.getState();

        // we went a little further, no longer accelerating.
        assertEquals(0.04, state2.x().x(), DELTA);
        assertEquals(1.00, state2.x().v(), DELTA);

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);

        stepTime();
        StateSE2 state3 = drive.getState();

        // a little further, but no longer accelerating
        assertEquals(0.06, state3.x().x(), DELTA);
        assertEquals(1.00, state3.x().v(), DELTA);

        drive.close();
    }
}
