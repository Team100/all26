package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.SimulatedGyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class FusedEstimatorTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void test0() throws IOException {
        // straight line motion computed by odometry and
        // also imputed via pose history
        double dt = TimedRobot100.LOOP_PERIOD_S;
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        FusedEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, false, layout, gyro, swerveLocal);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);

        stepTime();

        estimate.reset(new Pose2d(), IsotropicNoiseSE2.high());
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(),
                        new VelocitySE2(0, 0, 0)),
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.02, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.04, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 0), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.06, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(Takt.get()));

        Map<Double, SwerveState> all = estimate.all();
        assertEquals(5, all.size());
        List<SwerveState> states = new ArrayList<>(all.values());

        // derive velocities from the poses
        // first state doesn't move
        TestUtil.verify(
                new VelocitySE2(0, 0, 0),
                VelocitySE2.velocity(
                        states.get(0).state().pose(),
                        states.get(1).state().pose(), dt));
        // next states do move
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(1).state().pose(),
                        states.get(2).state().pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(2).state().pose(),
                        states.get(3).state().pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(3).state().pose(),
                        states.get(4).state().pose(), dt));

        drive.close();
    }

    @Test
    void test1() throws IOException {
        // test uniform twist motion computed by odometry and
        // also imputed via pose history
        double dt = TimedRobot100.LOOP_PERIOD_S;
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0, 0);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        FusedEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, false, layout, gyro, swerveLocal);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);

        stepTime();

        estimate.reset(new Pose2d(), IsotropicNoiseSE2.high());
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(),
                        new VelocitySE2(0, 0, 0)),
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 1), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.02, 0.001, new Rotation2d(0.02)),
                        new VelocitySE2(0.991, 0.064, 0.993)), // ???
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 1), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.04, 0.002, new Rotation2d(0.04)),
                        new VelocitySE2(1.0, 0.02, 1.0)),
                estimate.getState(Takt.get()));

        drive.setChassisVelocities(new ChassisVelocities(1, 0, 1), ChassisAcceleration.ZERO);
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.06, 0.002, new Rotation2d(0.06)),
                        new VelocitySE2(1.0, 0.04, 1.0)),
                estimate.getState(Takt.get()));

        Map<Double, SwerveState> all = estimate.all();
        assertEquals(5, all.size());
        List<SwerveState> states = new ArrayList<>(all.values());

        // derive velocities from the poses
        // first state doesn't move
        TestUtil.verify(
                new VelocitySE2(0, 0, 0),
                VelocitySE2.velocity(
                        states.get(0).state().pose(),
                        states.get(1).state().pose(), dt));
        // next states do move
        TestUtil.verify(
                new VelocitySE2(0.991, 0.064, 0.993), // ???
                VelocitySE2.velocity(
                        states.get(1).state().pose(),
                        states.get(2).state().pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0.02, 1),
                VelocitySE2.velocity(
                        states.get(2).state().pose(),
                        states.get(3).state().pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0.04, 1),
                VelocitySE2.velocity(
                        states.get(3).state().pose(),
                        states.get(4).state().pose(), dt));

        drive.close();
    }

    @Test
    void test2() throws IOException {
        // test uniform twist motion computed by odometry and
        // also imputed via pose history, to convince myself
        // that the imputation will work ok for the GTSAM case,
        // which doesn't (yet) include velocity.
        double dt = TimedRobot100.LOOP_PERIOD_S;
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        LoggerFactory fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        Gyro gyro = new SimulatedGyro(logger, swerveKinodynamics, collection, 0, 0);
        assertEquals(0, gyro.getYawNWU().getRadians(), DELTA);
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);

        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        FusedEstimator estimate = new FusedEstimator(
                logger, fieldLogger, swerveKinodynamics, false, layout, gyro, swerveLocal);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                logger,
                estimate,
                swerveLocal);

        stepTime();

        estimate.reset(new Pose2d(), IsotropicNoiseSE2.high());
        stepTime();
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(),
                        new VelocitySE2(0, 0, 0)),
                estimate.getState(Takt.get()));

        for (double t = 0.02; t < 1; t += 0.02) {
            drive.setChassisVelocities(new ChassisVelocities(1, 0, 1), ChassisAcceleration.ZERO);
            stepTime();
            double timestampS = Takt.get();
            if (DEBUG) {
                StateSE2 state0 = estimate.getState(timestampS - dt);
                System.out.printf("TIME [%6.3f] POSE [%s] VELOCITY [%s] GYRO [%s]\n",
                        timestampS - dt, state0.pose(), state0.velocity(), gyro.getYawNWU());
                StateSE2 state1 = estimate.getState(timestampS);
                System.out.printf("TIME [%6.3f] POSE [%s] VELOCITY [%s] GYRO [%s]\n",
                        timestampS, state1.pose(), state1.velocity(), gyro.getYawNWU());
                VelocitySE2 v = VelocitySE2.velocity(
                        state0.pose(),
                        state1.pose(), dt);
                System.out.printf("V [%s]\n", v);
            }
        }

        drive.close();
    }
}
