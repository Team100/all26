package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Rotation2d;

public class OdometryUpdaterTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private SwerveModulePositions positions;

    @Test
    void testNewState1() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions, UnaryOperator.identity());
        // previous state is at zero, but uncertain
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        // high bias sigma compared to the real value
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // measurements haven't moved
        Rotation2d gyroYaw = new Rotation2d();

        // result shouldn't move.
        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0, newState.state().pose().getX(), 1e-6);
        assertEquals(0, newState.state().pose().getY(), 1e-6);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), 1e-6);
        // variance shouldn't change.
        assertEquals(1, newState.noise().cartesian(), 1e-6);
        assertEquals(1.000004, newState.noise().rotation(), 1e-6);
        // there's no movement in yaw, so bias is unchanged.
        assertEquals(0, newState.gyroBias().mean(), 1e-6);
        // previous bias sigma was 0.001
        // the bias measurement adds the odometry variance (which is zero, it's not
        // moving) with the gyro measurement variance, which is the white noise
        // so the new bias estimate at this point just reflects the gyro
        // measurement noise.
        assertEquals(0.000943, newState.gyroBias().sigma(), 1e-6);
        // do it again
        newState = ou.newState(newState, 0.02, gyroYaw, positions);
        // now the gyro bias is improved
        assertEquals(0.000894, newState.gyroBias().sigma(), 1e-6);
        // a few more times
        for (int i = 0; i < 50; ++i) {
            newState = ou.newState(newState, 0.02, gyroYaw, positions);
            if (DEBUG)
                System.out.println(newState.gyroBias().sigma());
        }
        // now the gyro bias is improved further
        // note how slow this is
        assertEquals(0.000365, newState.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewState2() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions, UnaryOperator.identity());

        // previous state is at zero, pretty sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // 0.1m ahead (this is max speed)
        Rotation2d gyroYaw = new Rotation2d();
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)));

        // result shouldn't be 0.1 ahead
        // high speed means considerable added variance
        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0.1, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, newState.noise().cartesian(), DELTA);
        assertEquals(0.010, newState.noise().rotation(), DELTA);
        assertEquals(0, newState.gyroBias().mean(), 1e-6);
        // more variance here due to odometry noise
        assertEquals(0.000999, newState.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewStateWithBias() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions, UnaryOperator.identity());

        // previous state is at zero, pretty sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        // initial bias estimate is zero
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // odometry says we're not rotating, but the gyro thinks we are.
        // this is 0.02 rad in 0.02 s so the bias is 1 rad/s
        Rotation2d gyroYaw = new Rotation2d(0.02);
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)));

        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0.1, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        // odo is not that trustworthy since it's so fast
        assertEquals(0.019, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, newState.noise().cartesian(), DELTA);
        assertEquals(0.010, newState.noise().rotation(), DELTA);
        // nonzero bias mean, we're not confident enough to go straight to the right
        // answer
        // assertEquals(0.000004, newState.gyroBias().mean(), 1e-6);
        assertEquals(4.4e-7, newState.gyroBias().mean(), 1e-6);
        // much more bias noise
        // assertEquals(0.001039, newState.gyroBias().sigma(), 1e-6);
        assertEquals(0.001004, newState.gyroBias().sigma(), 1e-6);
        // Grind on the bias for awhile
        for (int i = 2; i < 100; ++i) {
            gyroYaw = new Rotation2d(0.02 * i);
            newState = ou.newState(newState, 0.02, gyroYaw, positions);
        }
        assertEquals(0.1, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        assertEquals(0.019, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, newState.noise().cartesian(), DELTA);
        assertEquals(0.031, newState.noise().rotation(), DELTA);
        // we get the right answer for the gyro bias
        assertEquals(0.999995, newState.gyroBias().mean(), 1e-6);
        // and we're getting more confident about it
        assertEquals(0.000289, newState.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewStateWithBias2() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, null, () -> positions, UnaryOperator.identity());

        // previous state is at zero, pretty sure.
        ModelSE2 sampleModel = new ModelSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.kZero();
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        SwerveState sample = new SwerveState(
                sampleModel, stateNoise, positions, yaw, bias);

        // much slower, so odometry is more trustworthy.
        Rotation2d gyroYaw = new Rotation2d(0.02);
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)));

        SwerveState newState = ou.newState(sample, 0.02, gyroYaw, positions);
        assertEquals(0.01, newState.state().pose().getX(), DELTA);
        assertEquals(0, newState.state().pose().getY(), DELTA);
        // we don't really believe the gyro
        assertEquals(0.019088, newState.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.010, newState.noise().cartesian(), DELTA);
        assertEquals(0.010, newState.noise().rotation(), DELTA);
        // much more bias
        // assertEquals(0.001307, newState.gyroBias().mean(), 1e-6);
        assertEquals(6.1e-5, newState.gyroBias().mean(), 1e-6);
        // a bit more bias noise
        // assertEquals(0.005206, newState.gyroBias().sigma(), 1e-6);
        assertEquals(0.001493, newState.gyroBias().sigma(), 1e-6);
    }

}
