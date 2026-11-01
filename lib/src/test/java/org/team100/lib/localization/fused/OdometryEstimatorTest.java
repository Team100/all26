package org.team100.lib.localization.fused;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Rotation2d;

public class OdometryEstimatorTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    @Test
    void testNewState1() {
        MockGyro gyro = new MockGyro();
        OdometryEstimator ou = new OdometryEstimator(
                log, kinodynamics, gyro.white_noise(),
                gyro.bias_noise(),
                false, true);
        // previous state is at zero, but uncertain
        StateSE2 sampleState = new StateSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(1, 1);
        final SwerveModulePositions positions = SwerveModulePositions.ZERO;
        final Rotation2d yaw = new Rotation2d();
        // high bias sigma compared to the real value
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        LocalizerEstimate estimate1 = new LocalizerEstimate(sampleState, stateNoise, bias);
        OdoGyroMeasurement measurement = new OdoGyroMeasurement(yaw, positions);
        double timestamp = 0;
        LocalizerEstimate estimate = ou.estimate(
                timestamp, estimate1, measurement, 0.02, measurement);
        assertEquals(0, estimate.state().pose().getX(), 1e-6);
        assertEquals(0, estimate.state().pose().getY(), 1e-6);
        assertEquals(0, estimate.state().pose().getRotation().getRadians(), 1e-6);
        // variance shouldn't change.
        assertEquals(1, estimate.noise().cartesian(), 1e-6);
        assertEquals(1.000004, estimate.noise().rotation(), 1e-6);
        // there's no movement in yaw, so bias is unchanged.
        assertEquals(0, estimate.gyroBias().mean(), 1e-6);
        // previous bias sigma was 0.001
        // the bias measurement adds the odometry variance (which is zero, it's not
        // moving) with the gyro measurement variance, which is the white noise
        // so the new bias estimate at this point just reflects the gyro
        // measurement noise.
        assertEquals(0.000943, estimate.gyroBias().sigma(), 1e-6);
        // do it again
        timestamp = 0.02;
        estimate = ou.estimate(
                timestamp, estimate, measurement, 0.04, measurement);
        // now the gyro bias is improved
        LocalizerEstimate estimate2 = estimate;
        assertEquals(0.000894, estimate2.gyroBias().sigma(), 1e-6);
        // a few more times
        for (int i = 0; i < 50; ++i) {
            estimate = ou.estimate(
                    timestamp, estimate, measurement, timestamp + 0.02, measurement);
            timestamp += 0.02;
            if (DEBUG) {
                System.out.println(estimate.gyroBias().sigma());
            }
        }
        // now the gyro bias is improved further
        // note how slow this is
        LocalizerEstimate estimate4 = estimate;
        assertEquals(0.000365, estimate4.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewState2() {
        MockGyro gyro = new MockGyro();
        OdometryEstimator ou = new OdometryEstimator(
                log, kinodynamics, gyro.white_noise(), gyro.bias_noise(),
                false, true);

        // previous state is at zero, pretty sure.
        StateSE2 sampleState = new StateSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.ZERO;
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        LocalizerEstimate estimate1 = new LocalizerEstimate(sampleState, stateNoise, bias);
        OdoGyroMeasurement measurement1 = new OdoGyroMeasurement(yaw, positions);

        // 0.1m ahead (this is max speed)
        Rotation2d gyroYaw = new Rotation2d();
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)));

        // result shouldn't be 0.1 ahead
        // high speed means considerable added variance
        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(gyroYaw, positions);
        LocalizerEstimate estimate = ou.estimate(
                0, estimate1, measurement1, 0.02, newMeasurement);
        assertEquals(0.1, estimate.state().pose().getX(), DELTA);
        assertEquals(0, estimate.state().pose().getY(), DELTA);
        assertEquals(0, estimate.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, estimate.noise().cartesian(), DELTA);
        assertEquals(0.010, estimate.noise().rotation(), DELTA);
        assertEquals(0, estimate.gyroBias().mean(), 1e-6);
        // more variance here due to odometry noise
        assertEquals(0.000999, estimate.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewStateWithBias() {
        MockGyro gyro = new MockGyro();
        OdometryEstimator ou = new OdometryEstimator(
                log, kinodynamics, gyro.white_noise(),
                gyro.bias_noise(),
                false, true);

        // previous state is at zero, pretty sure.
        StateSE2 sampleState = new StateSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.ZERO;
        Rotation2d yaw = new Rotation2d();
        // initial bias estimate is zero
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        LocalizerEstimate estimate1 = new LocalizerEstimate(sampleState, stateNoise, bias);
        OdoGyroMeasurement measurement1 = new OdoGyroMeasurement(yaw, positions);

        // odometry says we're not rotating, but the gyro thinks we are.
        // this is 0.02 rad in 0.02 s so the bias is 1 rad/s
        Rotation2d gyroYaw = new Rotation2d(0.02);
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.kZero)));

        double timestamp = 0;
        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(gyroYaw, positions);
        LocalizerEstimate estimate = ou.estimate(
                timestamp, estimate1, measurement1, 0.02, newMeasurement);
        timestamp += 0.02;
        OdoGyroMeasurement measurement = new OdoGyroMeasurement(gyroYaw, positions);
        assertEquals(0.1, estimate.state().pose().getX(), DELTA);
        assertEquals(0, estimate.state().pose().getY(), DELTA);
        // odo is not that trustworthy since it's so fast
        assertEquals(0.019, estimate.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, estimate.noise().cartesian(), DELTA);
        assertEquals(0.010, estimate.noise().rotation(), DELTA);
        // nonzero bias mean, we're not confident enough to go straight to the right
        // answer
        // assertEquals(0.000004, newState.gyroBias().mean(), 1e-6);
        assertEquals(4.4e-7, estimate.gyroBias().mean(), 1e-6);
        // much more bias noise
        // assertEquals(0.001039, newState.gyroBias().sigma(), 1e-6);
        assertEquals(0.001004, estimate.gyroBias().sigma(), 1e-6);
        // Grind on the bias for awhile
        for (int i = 2; i < 100; ++i) {
            gyroYaw = new Rotation2d(0.02 * i);
            OdoGyroMeasurement measurement2 = new OdoGyroMeasurement(gyroYaw, positions);
            estimate = ou.estimate(
                    timestamp, estimate, measurement, timestamp + 0.02, measurement2);
            timestamp += 0.02;
            measurement = measurement2;
        }
        assertEquals(0.1, estimate.state().pose().getX(), DELTA);
        assertEquals(0, estimate.state().pose().getY(), DELTA);
        assertEquals(0.019, estimate.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.014, estimate.noise().cartesian(), DELTA);
        assertEquals(0.031, estimate.noise().rotation(), DELTA);
        // we get the right answer for the gyro bias
        assertEquals(0.999995, estimate.gyroBias().mean(), 1e-6);
        // and we're getting more confident about it
        assertEquals(0.000289, estimate.gyroBias().sigma(), 1e-6);
    }

    @Test
    void testNewStateWithBias2() {
        MockGyro gyro = new MockGyro();
        OdometryEstimator ou = new OdometryEstimator(
                log, kinodynamics, gyro.white_noise(),
                gyro.bias_noise(),
                false, true);

        // previous state is at zero, pretty sure.
        StateSE2 sampleState = new StateSE2();
        IsotropicNoiseSE2 stateNoise = IsotropicNoiseSE2.fromStdDev(0.01, 0.01);
        SwerveModulePositions positions = SwerveModulePositions.ZERO;
        Rotation2d yaw = new Rotation2d();
        VariableR1 bias = VariableR1.fromStdDev(0, 0.001);
        LocalizerEstimate estimate1 = new LocalizerEstimate(sampleState, stateNoise, bias);
        OdoGyroMeasurement measurement1 = new OdoGyroMeasurement(yaw, positions);

        // much slower, so odometry is more trustworthy.
        Rotation2d gyroYaw = new Rotation2d(0.02);
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.01, Optional.of(Rotation2d.kZero)));

        LocalizerEstimate estimate = ou.estimate(0,
                estimate1, measurement1,
                0.02, new OdoGyroMeasurement(gyroYaw, positions));
        assertEquals(0.01, estimate.state().pose().getX(), DELTA);
        assertEquals(0, estimate.state().pose().getY(), DELTA);
        // we don't really believe the gyro
        assertEquals(0.019088, estimate.state().pose().getRotation().getRadians(), DELTA);
        assertEquals(0.010, estimate.noise().cartesian(), DELTA);
        assertEquals(0.010, estimate.noise().rotation(), DELTA);
        // much more bias
        // assertEquals(0.001307, newState.gyroBias().mean(), 1e-6);
        assertEquals(6.1e-5, estimate.gyroBias().mean(), 1e-6);
        // a bit more bias noise
        // assertEquals(0.005206, newState.gyroBias().sigma(), 1e-6);
        assertEquals(0.001493, estimate.gyroBias().sigma(), 1e-6);
    }

}
