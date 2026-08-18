package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.TreeMap;
import java.util.function.UnaryOperator;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleState100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DataLogManager;

class SwerveDrivePoseEstimator100Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    private final SwerveModulePosition100 p0 = new SwerveModulePosition100(0, Optional.of(Rotation2d.kZero));
    /** at the origin, facing +x */
    private final SwerveModulePositions positionZero = new SwerveModulePositions(p0, p0, p0, p0);
    private final SwerveModulePosition100 p01 = new SwerveModulePosition100(0.1,
            Optional.of(Rotation2d.kZero));
    /** 0.1 m ahead */
    private final SwerveModulePositions position01 = new SwerveModulePositions(p01, p01, p01, p01);
    /** 1 meter ahead */
    private final Pose2d visionPose = new Pose2d(1, 0, Rotation2d.kZero);

    private SwerveModulePositions positions;

    private static void verify(double x, double sigma, SwerveHistory history, double timestamp) {
        SwerveState state = history.getRecord(timestamp);
        ModelSE2 model = state.state();
        Pose2d estimate = model.pose();
        assertEquals(x, estimate.getX(), DELTA);
        assertEquals(0, estimate.getY(), DELTA);
        assertEquals(0, estimate.getRotation().getRadians(), DELTA);
        IsotropicNoiseSE2 noise = state.noise();
        assertEquals(sigma, noise.cartesian(), DELTA);
    }

    private static void verifyBias(double mean, double sigma, SwerveHistory history, double timestamp) {
        assertEquals(mean, history.getRecord(timestamp).gyroBias().mean(), DELTA);
        assertEquals(sigma, history.getRecord(timestamp).gyroBias().sigma(), DELTA);
    }

    private static void verifyVelocity(double xV, ModelSE2 state) {
        VelocitySE2 v = state.velocity();
        assertEquals(xV, v.x(), DELTA);
    }

    @BeforeEach
    void nolog() {
        DataLogManager.stop();
    }

    @Test
    void testGyroOffset() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);
        Pose2d p = history.apply(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        assertEquals(0, p.getRotation().getRadians(), DELTA);
        // assertEquals(0, ou.getGyroOffset().getRadians(), DELTA);
        // force the pose to rotate 90
        ou.reset(new Pose2d(0, 0, Rotation2d.kCCW_90deg), IsotropicNoiseSE2.high(), 0);
        p = history.apply(0).pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        // and we get that back
        assertEquals(Math.PI / 2, p.getRotation().getRadians(), DELTA);
        // and the offset is correct since the gyro itself didn't change.
        // assertEquals(Math.PI / 2, ou.getGyroOffset().getRadians(), DELTA);
    }

    @Test
    void odo1() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // move 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, 0.1, history, 0.02);
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update
        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.00, noisyMeasurement);
        // position slides over there
        verify(0.038, 0.102, history, 0.00);
        verifyBias(0, 1, history, 0);
        verifyVelocity(0.000, history.apply(0.00));
        // odometry adds to that
        verify(0.138, 0.102, history, 0.02);
        // velocity is unchanged.
        verifyVelocity(5.000, history.apply(0.02));
    }

    @Test
    void odo2() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, 0.1, history, 0.02);
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update, but later
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.02, noisyMeasurement);
        // initial position is unchanged
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verifyVelocity(0.000, history.apply(0.00));
        // not sure what's happening here
        verify(0.135, 0.102, history, 0.02);
        // velocity is STILL unchanged, i.e. not consistent with the pose history, which
        // is probably better
        // than making velocity reflect the camera noise.
        verifyVelocity(5.000, history.apply(0.02));
    }

    @Test
    void odo3() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // 0.1 m
        positions = position01;
        ou.update(0.02);
        verify(0.000, 0.1, history, 0.00);
        verifyVelocity(0.000, history.apply(0.00));
        // velocity is the delta
        verify(0.1, 0.1, history, 0.02);
        verifyVelocity(5.000, history.apply(0.02));

        // big vision update, even later
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionNoise);
        vu.put(0.04, noisyMeasurement);
        // initial position is unchanged
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verifyVelocity(0.000, history.apply(0.00));
        // camera does nothing
        verify(0.1, 0.1, history, 0.02);
        // velocity is STILL unchanged, i.e. not consistent with the post history, which
        // is probably better
        // than making velocity reflect the camera noise.
        verifyVelocity(5.000, history.apply(0.02));
        // this is 0.1 towards the camera 1.0
        verify(0.135, 0.102, history, 0.04);
        // still velo is unaffected
        verifyVelocity(5.000, history.apply(0.04));
    }

    @Test
    void outOfOrder() {
        // out of order vision updates
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        // initial pose = 0
        verify(0, 0.1, history, 0.00);

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.01);
        verify(0.000, 0.1, history, 0.02);

        // now vision says we're one meter away, so pose goes towards that
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.01, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.038, 0.101, history, 0.01);

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.get());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        positions = positionZero;
        ou.update(0.02);

        verify(0.000, 0.1, history, 0.00);
        verify(0.038, 0.101, history, 0.01);
        verify(0.038, 0.101, history, 0.02);

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        positions = position01;
        ou.update(0.04);

        verify(0.000, 0.1, history, 0.00);
        verify(0.038, 0.102, history, 0.02);
        verify(0.138, 0.102, history, 0.04);

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        vu.put(0.015, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.077, 0.103, history, 0.015);
        // odometry thinks no motion at 0.02 so repeat the vision estimate here
        verify(0.077, 0.103, history, 0.02);
        // odometry of 0.1 + the vision estimate from 0.02.
        verify(0.177, 0.104, history, 0.04);

        // wheels are in the same position as the previous iteration,
        positions = position01;
        ou.update(0.06);
        verify(0.000, 0.1, history, 0.00);
        verify(0.077, 0.103, history, 0.02);
        verify(0.177, 0.104, history, 0.04);
        verify(0.177, 0.104, history, 0.06);
        verify(0.177, 0.104, history, 0.08);

        // a little earlier than the previous estimate does nothing
        vu.put(0.014, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        // notices the vision input a bit earlier
        verify(0.077, 0.104, history, 0.014);
        // but doesn't change this estimate since it's the same, and we're not moving,
        // we don't replay vision input
        // it would be better if two vision estimates pulled harder than one,
        // even if they come in out-of-order.
        verify(0.077, 0.104, history, 0.015);
        verify(0.077, 0.104, history, 0.02);
        verify(0.177, 0.105, history, 0.04);
        verify(0.177, 0.105, history, 0.06);

        // a little later than the previous estimate works normally.
        vu.put(0.016, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.077, 0.104, history, 0.014);
        verify(0.077, 0.104, history, 0.015);
        // drag the pose towards the vision estimate a bit.
        verify(0.116, 0.105, history, 0.016);
        verify(0.116, 0.105, history, 0.02);
        verify(0.216, 0.106, history, 0.04);
        verify(0.216, 0.106, history, 0.06);

        // wheels not moving -> no change,
        positions = position01;
        ou.update(0.08);
        verify(0.000, 0.1, history, 0.00);
        verify(0.116, 0.105, history, 0.02);
        verify(0.216, 0.106, history, 0.04);
        verify(0.216, 0.106, history, 0.06);
        verify(0.216, 0.106, history, 0.08);
    }

    @Test
    void minorWeirdness() {
        // weirdness with out-of-order vision updates

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        // initial pose = 0
        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.
        positions = positionZero;
        ou.update(0.0);
        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        // now vision says we're one meter away, so pose goes towards that
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.01, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.038, 0.102, history, 0.02);
        verify(0.038, 0.102, history, 0.04);
        verify(0.038, 0.102, history, 0.06);
        verify(0.038, 0.102, history, 0.08);

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.apply());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        positions = positionZero;
        ou.update(0.02);
        verify(0.000, 0.1, history, 0.00);
        verify(0.038, 0.102, history, 0.02);
        verify(0.038, 0.102, history, 0.04);
        verify(0.038, 0.102, history, 0.06);
        verify(0.038, 0.102, history, 0.08);

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        positions = position01;
        ou.update(0.04);
        verify(0.000, 0.1, history, 0.00);
        verify(0.038, 0.102, history, 0.02);
        verify(0.138, 0.102, history, 0.04);
        verify(0.138, 0.102, history, 0.06);
        verify(0.138, 0.102, history, 0.08);

        // here's the delayed update from above, which moves the estimate and
        // then the odometry is applied on top of that.
        vu.put(0.015, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.077, 0.103, history, 0.02);
        verify(0.177, 0.103, history, 0.04);
        verify(0.177, 0.103, history, 0.06);
        verify(0.177, 0.103, history, 0.08);

        // wheels are in the same position as the previous iteration
        positions = position01;
        ou.update(0.06);
        verify(0.000, 0.1, history, 0.00);
        verify(0.077, 0.103, history, 0.02);
        verify(0.177, 0.103, history, 0.04);
        verify(0.177, 0.103, history, 0.06);
        verify(0.177, 0.103, history, 0.08);

        // a little earlier than the previous estimate does nothing.
        vu.put(0.014, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.077, 0.104, history, 0.02);
        verify(0.177, 0.104, history, 0.04);
        verify(0.177, 0.104, history, 0.06);
        verify(0.177, 0.104, history, 0.08);

        // a little later than the previous estimate works normally.
        vu.put(0.016, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.116, 0.105, history, 0.02);
        verify(0.216, 0.105, history, 0.04);
        verify(0.216, 0.105, history, 0.06);
        verify(0.216, 0.105, history, 0.08);

        // wheels not moving -> no change
        positions = position01;
        ou.update(0.08);
        verify(0.000, 0.1, history, 0.00);
        verify(0.116, 0.105, history, 0.02);
        verify(0.216, 0.105, history, 0.04);
        verify(0.216, 0.105, history, 0.06);
        verify(0.216, 0.105, history, 0.08);
    }

    @Test
    void test0105() {
        // this is the current (post-comp 2024) base case.
        // within a few frames, the estimate converges on the vision input.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        positions = positionZero;
        ou.update(0);
        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.02, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.038, 0.102, history, 0.02);
        verify(0.038, 0.102, history, 0.04);
        verify(0.038, 0.102, history, 0.06);
        verify(0.038, 0.102, history, 0.08);

        vu.put(0.04, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.038, 0.102, history, 0.02);
        verify(0.077, 0.103, history, 0.04);
        verify(0.077, 0.103, history, 0.06);
        verify(0.077, 0.103, history, 0.08);

        vu.put(0.06, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.038, 0.102, history, 0.02);
        verify(0.077, 0.103, history, 0.04);
        verify(0.114, 0.104, history, 0.06);
        verify(0.114, 0.104, history, 0.08);
    }

    @Test
    void test0110() {
        // double vision stdev (r) -> slower convergence
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(1.0, Double.MAX_VALUE);
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.1, 0.1), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        positions = positionZero;
        ou.update(0);
        verify(0.000, 0.1, history, 0.00);
        verify(0.000, 0.1, history, 0.02);
        verify(0.000, 0.1, history, 0.04);
        verify(0.000, 0.1, history, 0.06);
        verify(0.000, 0.1, history, 0.08);

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.02, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.100, history, 0.02);
        verify(0.010, 0.100, history, 0.04);
        verify(0.010, 0.100, history, 0.06);
        verify(0.010, 0.100, history, 0.08);

        vu.put(0.04, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.100, history, 0.02);
        verify(0.020, 0.100, history, 0.04);
        verify(0.020, 0.100, history, 0.06);
        verify(0.020, 0.100, history, 0.08);

        vu.put(0.06, noisyMeasurement);
        verify(0.000, 0.1, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.100, history, 0.02);
        verify(0.020, 0.100, history, 0.04);
        verify(0.029, 0.101, history, 0.06);
        verify(0.029, 0.101, history, 0.08);
    }

    @Test
    void test00505() {
        // half odo stdev (q) -> slower convergence
        // the K is q/(q+qr) so it's q compared to r that matters.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        positions = positionZero;
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.05, 0.05), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        verify(0.000, 0.05, history, 0.00);
        verify(0.000, 0.05, history, 0.02);
        verify(0.000, 0.05, history, 0.04);
        verify(0.000, 0.05, history, 0.06);
        verify(0.000, 0.05, history, 0.08);

        positions = positionZero;
        ou.update(0);
        verify(0.000, 0.05, history, 0.00);
        verify(0.000, 0.05, history, 0.02);
        verify(0.000, 0.05, history, 0.04);
        verify(0.000, 0.05, history, 0.06);
        verify(0.000, 0.05, history, 0.08);

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.02, noisyMeasurement);
        verify(0.000, 0.05, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.052, history, 0.02);
        verify(0.010, 0.052, history, 0.04);
        verify(0.010, 0.052, history, 0.06);
        verify(0.010, 0.052, history, 0.08);

        vu.put(0.04, noisyMeasurement);
        verify(0.000, 0.05, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.052, history, 0.02);
        verify(0.020, 0.053, history, 0.04);
        verify(0.020, 0.053, history, 0.06);
        verify(0.020, 0.053, history, 0.08);

        vu.put(0.06, noisyMeasurement);
        verify(0.000, 0.05, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.052, history, 0.02);
        verify(0.020, 0.053, history, 0.04);
        verify(0.031, 0.055, history, 0.06);
        verify(0.031, 0.055, history, 0.08);
    }

    @Test
    void reasonable() {
        // stdev that actually make sense
        // actual odometry error is very low
        // measured camera error is something under 10 cm
        // these yield much slower convergence, maybe too slow? try and see.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        Gyro gyro = new MockGyro();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.1, Double.MAX_VALUE);
        SwerveHistory history = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, history, () -> positions, UnaryOperator.identity());
        positions = positionZero;
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.fromStdDev(0.01, 0.01), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, history, ou);

        verify(0.000, 0.01, history, 0.00);
        verify(0.000, 0.01, history, 0.02);
        verify(0.000, 0.01, history, 0.04);
        verify(0.000, 0.01, history, 0.06);
        verify(0.000, 0.01, history, 0.08);

        positions = positionZero;
        ou.update(0);
        verify(0.000, 0.01, history, 0.00);
        verify(0.000, 0.01, history, 0.02);
        verify(0.000, 0.01, history, 0.04);
        verify(0.000, 0.01, history, 0.06);
        verify(0.000, 0.01, history, 0.08);

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vu.put(0.02, noisyMeasurement);
        verify(0.000, 0.01, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.017, history, 0.02);
        verify(0.010, 0.017, history, 0.04);
        verify(0.010, 0.017, history, 0.06);
        verify(0.010, 0.017, history, 0.08);

        vu.put(0.04, noisyMeasurement);
        verify(0.000, 0.01, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.017, history, 0.02);
        verify(0.038, 0.029, history, 0.04);
        verify(0.038, 0.029, history, 0.06);
        verify(0.038, 0.029, history, 0.08);

        vu.put(0.06, noisyMeasurement);
        verify(0.000, 0.01, history, 0.00);
        verifyBias(0, 1, history, 0);
        verify(0.010, 0.017, history, 0.02);
        verify(0.038, 0.029, history, 0.04);
        verify(0.112, 0.046, history, 0.06);
        verify(0.112, 0.046, history, 0.08);

    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    // used below
    State groundTruthState = new State();
    Random rand = new Random(3538);
    Trajectory trajectory = new Trajectory();

    @Test
    void testAccuracyFacingTrajectory() {

        trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45)),
                        new Pose2d(3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(135)),
                        new Pose2d(-3, 0, Rotation2d.fromDegrees(-90)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(45))),
                new TrajectoryConfig(2, 2));
        groundTruthState = trajectory.sample(0);

        Gyro gyro = new Gyro() {
            @Override
            public double white_noise() {
                // the noise level is enormous
                return 0.05 / Math.sqrt(0.02);
            }

            @Override
            public double bias_noise() {
                return 1e-5;
            }

            @Override
            public Rotation2d getYawNWU() {
                // yaw is a noisy version of ground truth
                double gyroNoiseSigma = 0.05;
                Rotation2d noise = new Rotation2d(rand.nextGaussian() * gyroNoiseSigma);
                Rotation2d yaw = groundTruthState.poseMeters.getRotation()
                        .plus(noise);
                // .minus(trajectory.getInitialPose().getRotation());
                // System.out.printf("yaw %s\n", yaw);
                return yaw;
            }

            @Override
            public double getYawRateNWU() {
                return 0.0;
            }

            @Override
            public Rotation2d getPitchNWU() {
                return null;
            }

            @Override
            public Rotation2d getRollNWU() {
                return null;
            }

            @Override
            public void periodic() {
            }
        };
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, 0.5);
        SwerveHistory estimator = new SwerveHistory(
                logger,
                kinodynamics,
                1, // extra long
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, estimator, () -> positions, UnaryOperator.identity());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());
        ou.reset(new Pose2d(), IsotropicNoiseSE2.high(), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, estimator, ou);

        Pose2d endingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(45));

        // new starting pose here, so we don't actually use the earlier initial pose

        ou.reset(
                trajectory.getInitialPose(),
                IsotropicNoiseSE2.high(),
                0);

        double t = 0.0;

        TreeMap<Double, Pose2d> visionUpdateQueue = new TreeMap<>();

        double maxError = Double.NEGATIVE_INFINITY;
        double errorSum = 0;

        double DT = 0.02;
        while (t <= trajectory.getTotalTimeSeconds()) {
            groundTruthState = trajectory.sample(t);

            // We are due for a new vision measurement if it's been 0.1
            // seconds since the last vision measurement
            double visionUpdateRate = 0.1;
            if (visionUpdateQueue.isEmpty() || visionUpdateQueue.lastKey() + visionUpdateRate < t) {
                // the vision update is a noisy version of ground truth, a lot of noise.
                Transform2d noise = new Transform2d(
                        new Translation2d(rand.nextGaussian() * 0.1, rand.nextGaussian() * 0.1),
                        new Rotation2d(rand.nextGaussian() * 0.05));
                Pose2d newVisionPose = groundTruthState.poseMeters.plus(noise);
                visionUpdateQueue.put(t, newVisionPose);
            }

            // We should apply the oldest vision measurement if it has been
            // `visionUpdateDelay` seconds since it was measured
            double visionDelay = 0.25;
            if (!visionUpdateQueue.isEmpty() && visionUpdateQueue.firstKey() + visionDelay < t) {
                var visionEntry = visionUpdateQueue.pollFirstEntry();
                Double timestamp = visionEntry.getKey();
                // System.out.printf("===== vision update at t=%f for timestamp %f\n",
                // t, timestamp);
                vu.put(
                        timestamp,
                        new NoisyPose2d(
                                visionEntry.getValue(),
                                visionMeasurementStdDevs));
            }

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    groundTruthState.velocityMetersPerSecond,
                    0,
                    groundTruthState.velocityMetersPerSecond * groundTruthState.curvatureRadPerMeter);

            SwerveModuleStates moduleStates = kinodynamics.getKinematics()
                    .inverse(SwerveKinodynamics.discretize(chassisSpeeds, DT));
            SwerveModuleState100[] moduleStatesAll = moduleStates.all();
            SwerveModulePosition100[] positionsAll = positions.all();
            SwerveModulePosition100[] newPositions = new SwerveModulePosition100[positionsAll.length];

            for (int i = 0; i < moduleStatesAll.length; i++) {
                // speed noise results in distance noise for each wheel (a lot of noise)
                double velocityNoise = 1 - rand.nextGaussian() * 0.05;
                double distanceMeters = positionsAll[i].distanceMeters()
                        + moduleStatesAll[i].speed() * velocityNoise * DT;

                Optional<Rotation2d> angle = moduleStatesAll[i].angle();
                Optional<Rotation2d> newAngle = Optional.empty();
                if (angle.isPresent()) {
                    // wheel steering noise
                    double angleNoise = rand.nextGaussian() * 0.005;
                    newAngle = Optional.of(
                            new Rotation2d(angle.get().getRadians() + angleNoise));
                }
                newPositions[i] = new SwerveModulePosition100(distanceMeters, newAngle);
            }

            positions = new SwerveModulePositions(newPositions[0], newPositions[1], newPositions[2], newPositions[3]);

            ou.update(t);
            ModelSE2 xHat = estimator.apply(t);

            double error = groundTruthState.poseMeters.getTranslation().getDistance(
                    xHat.pose().getTranslation());
            // System.out.printf("error %f\n", error);
            if (error > maxError) {
                maxError = error;
            }
            errorSum += error;

            if (DEBUG) {
                System.out.printf("t %4.2f GT (%6.3f, %6.3f, %6.3f) xhat (%6.3f, %6.3f, %6.3f)\n",
                        t,
                        groundTruthState.poseMeters.getX(),
                        groundTruthState.poseMeters.getY(),
                        groundTruthState.poseMeters.getRotation().getRadians(),
                        xHat.pose().getX(),
                        xHat.pose().getY(),
                        xHat.pose().getRotation().getRadians());
            }

            t += DT;

        }

        Pose2d estimatedEndingPose = estimator.apply(t).pose();
        assertEquals(
                endingPose.getX(), estimatedEndingPose.getX(), 0.08, "Incorrect Final X");
        assertEquals(
                endingPose.getY(), estimatedEndingPose.getY(), 0.08, "Incorrect Final Y");
        assertEquals(
                endingPose.getRotation().getRadians(),
                estimatedEndingPose.getRotation().getRadians(),
                0.15,
                "Incorrect Final Theta");

        assertEquals(
                0.0, errorSum / (trajectory.getTotalTimeSeconds() / DT), 0.09, "Incorrect mean error");
        assertEquals(0.0, maxError, 0.2, "Incorrect max error");

    }

    @Test
    void testSimultaneousVisionMeasurements() {
        // This tests for multiple vision measurements appled at the same time. The
        // expected behavior is that all measurements affect the estimated pose. The
        // alternative result is that only one vision measurement affects the outcome.
        // If that were the case, after 1000 measurements, the estimated pose would
        // converge to that measurement.

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();
        Gyro gyro = new MockGyro();

        final SwerveModulePosition100 fl = new SwerveModulePosition100();
        final SwerveModulePosition100 fr = new SwerveModulePosition100();
        final SwerveModulePosition100 bl = new SwerveModulePosition100();
        final SwerveModulePosition100 br = new SwerveModulePosition100();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.9, 0.9);
        SwerveHistory estimator = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, estimator, () -> positions, UnaryOperator.identity());
        positions = new SwerveModulePositions(fl, fr, bl, br);
        ou.reset(new Pose2d(1, 2, Rotation2d.fromDegrees(270)), IsotropicNoiseSE2.high(), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, estimator, ou);

        ou.update(0);

        var visionMeasurements = new Pose2d[] {
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 1, Rotation2d.fromDegrees(90)),
                new Pose2d(2, 4, Rotation2d.fromRadians(180)),
        };

        for (int i = 0; i < 1000; i++) {
            for (var measurement : visionMeasurements) {
                vu.put(0.00, new NoisyPose2d(
                        measurement, visionMeasurementStdDevs));
            }
        }

        for (var measurement : visionMeasurements) {
            // at time zero the whole time
            Pose2d estimatedPose = estimator.apply(0).pose();
            var dx = Math.abs(measurement.getX() - estimatedPose.getX());
            var dy = Math.abs(measurement.getY() - estimatedPose.getY());
            var dtheta = Math.abs(
                    measurement.getRotation().getDegrees()
                            - estimatedPose.getRotation().getDegrees());

            assertTrue(dx > 0.08 || dy > 0.08 || dtheta > 0.08);
        }
    }

    @Test
    void testDiscardsOldVisionMeasurements() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forWPITest();
        Gyro gyro = new MockGyro();

        var estimator = new SwerveHistory(
                logger,
                kinodynamics,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0); // zero initial time

        OdometryUpdater ou = new OdometryUpdater(
                logger, kinodynamics, gyro, estimator, () -> positions, UnaryOperator.identity());
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());
        ou.reset(Pose2d.kZero, IsotropicNoiseSE2.high(), 0);
        NudgingVisionUpdater vu = new NudgingVisionUpdater(logger, estimator, ou);

        double time = 0;

        // Add enough measurements to fill up the buffer
        for (; time < 4; time += 0.02) {
            ou.update(time);
        }

        Pose2d odometryPose = estimator.apply(time).pose();

        // Apply a vision measurement made 3 seconds ago
        // This test passes if this does not cause a ConcurrentModificationException.
        vu.put(
                1,
                new NoisyPose2d(
                        new Pose2d(new Translation2d(10, 10), new Rotation2d(0.1)),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        Pose2d visionPose = estimator.apply(time).pose();

        assertEquals(odometryPose.getX(), visionPose.getX(), DELTA);
        assertEquals(odometryPose.getY(), visionPose.getY(), DELTA);
        assertEquals(
                odometryPose.getRotation().getRadians(),
                visionPose.getRotation().getRadians(), DELTA);
    }
}