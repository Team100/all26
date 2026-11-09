package org.team100.lib.localization.fused;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class FusedEstimatorTest {
    private static final double DELTA = 0.001;

    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    /** at the origin, facing +x */
    private final SwerveModulePosition100 p01 = new SwerveModulePosition100(0.1,
            Optional.of(Rotation2d.kZero));
    /** 0.1 m ahead */
    private final SwerveModulePositions position01 = new SwerveModulePositions(p01, p01, p01, p01);
    /** 1 meter ahead */
    private final Pose2d visionPose = new Pose2d(1, 0, Rotation2d.kZero);

    private static void verify(double x, double sigma, NoisyStateSE2 ns) {
        StateSE2 state = ns.state();
        Pose2d pose = state.pose();
        assertEquals(x, pose.getX(), DELTA);
        assertEquals(0, pose.getY(), DELTA);
        assertEquals(0, pose.getRotation().getRadians(), DELTA);
        IsotropicNoiseSE2 noise = ns.noise();
        assertEquals(sigma, noise.cartesian(), DELTA);
    }

    private static void verifyVelocity(double xV, NoisyStateSE2 state) {
        VelocitySE2 v = state.velocity();
        assertEquals(xV, v.x(), DELTA);
    }

    private Rotation2d yaw;
    private SwerveModulePositions positions;
    private List<VisionMeasurement> vision;

    @Test
    void testDump() throws IOException {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        NoisyPose2d measurement = new NoisyPose2d(new Pose2d(), IsotropicNoiseSE2.fromStdDev(1, 1));

        OdometryEstimator odometryEstimate = new OdometryEstimator(
                log,
                kinodynamics,
                0.04,
                0.005,
                false,
                false);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(log);
        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimate,
                visionEstimate);

        estimator.update(0.00);
        vision = List.of(new VisionMeasurement(0.01, measurement));
        estimator.update(0.02);
        estimator.dump();
    }

    @Test
    void test0() throws IOException {
        // straight line motion computed by odometry and
        // also imputed via pose history
        double dt = TimedRobot100.LOOP_PERIOD_S;
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        OdometryEstimator odometryEstimate = new OdometryEstimator(
                logger,
                swerveKinodynamics,
                0,
                1e-5,
                false,
                false);
        NudgingVisionEstimator visionEstimate = new NudgingVisionEstimator(logger);
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        FusedEstimator estimate = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimate,
                visionEstimate);

        estimate.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(),
                        new VelocitySE2(0, 0, 0)),
                estimate.getState(0).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)));
        estimate.update(0.02);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.02, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.02).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.04, Optional.of(Rotation2d.kZero)));
        estimate.update(0.04);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.04, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.04).state());

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.06, Optional.of(Rotation2d.kZero)));
        estimate.update(0.06);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.06, 0, Rotation2d.kZero),
                        new VelocitySE2(1.0, 0, 0)),
                estimate.getState(0.06).state());

        Map<Double, NoisyStateSE2> all = estimate.all();
        assertEquals(4, all.size());
        List<NoisyStateSE2> states = new ArrayList<>(all.values());

        // derive velocities from the poses
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(0).pose(),
                        states.get(1).pose(), dt));
        // next states do move
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(1).pose(),
                        states.get(2).pose(), dt));
        TestUtil.verify(
                new VelocitySE2(1, 0, 0),
                VelocitySE2.velocity(
                        states.get(2).pose(),
                        states.get(3).pose(), dt));
    }

    @Test
    void odo1() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);
        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // move 0.1 m

        positions = position01;
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // velocity is the delta
        verify(0.1, 0.1, estimator.getState(0.02));
        verifyVelocity(5.000, estimator.getState(0.02));

        // big vision update
        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0, noisyMeasurement));
        estimator.update(0.02);
        vision = List.of();

        // position slides over there
        verify(0.038, 0.102, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // odometry adds to that
        verify(0.138, 0.102, estimator.getState(0.02));
        // velocity is unchanged.
        verifyVelocity(5.000, estimator.getState(0.02));
    }

    @Test
    void odo2() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));

        // 0.1 m

        positions = position01;
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // velocity is the delta
        verify(0.1, 0.1, estimator.getState(0.02));
        verifyVelocity(5.000, estimator.getState(0.02));

        // big vision update, but later
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);
        vision = List.of(new VisionMeasurement(0.02, noisyMeasurement));
        estimator.update(0.02);

        // initial position is unchanged
        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // not sure what's happening here
        verify(0.135, 0.102, estimator.getState(0.02));
        // velocity is unchanged
        verifyVelocity(5.000, estimator.getState(0.02));
    }

    @Test
    void odo3() {
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();
        IsotropicNoiseSE2 visionNoise = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);
        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);
        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));
        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));

        // 0.1 m
        positions = position01;
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // velocity is the delta
        verify(0.1, 0.1, estimator.getState(0.02));
        verifyVelocity(5.000, estimator.getState(0.02));

        // big vision update, even later
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionNoise);

        vision = List.of(new VisionMeasurement(0.04, noisyMeasurement));
        estimator.update(0.04);

        // initial position is unchanged
        // LocalizerEstimate estimate5 = estimator.getState(0.00);
        verify(0.000, 0.1, estimator.getState(0.00));
        verifyVelocity(0.000, estimator.getState(0.00));
        // camera does nothing
        verify(0.1, 0.1, estimator.getState(0.02));
        // velocity is unchanged
        verifyVelocity(5.000, estimator.getState(0.02));
        // this is 0.1 towards the camera 1.0
        verify(0.135, 0.102, estimator.getState(0.04));
        // because the "update" looks at both odo and vision inputs, and
        // because we didn't change the wheel positions, v is zero.
        verifyVelocity(0.000, estimator.getState(0.04));
    }

    @Test
    void outOfOrder() {
        // out of order vision updates
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        // initial pose = 0
        verify(0, 0.1, estimator.getState(0.00));

        // pose stays zero when updated at time zero
        // if we try to update zero, there's nothing to compare it to,
        // so we should just ignore this update.

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.000, 0.1, estimator.getState(0.01));
        verify(0.000, 0.1, estimator.getState(0.02));

        // now vision says we're one meter away, so pose goes towards that
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.01, noisyMeasurement));
        estimator.update(0.01);
        vision = List.of();

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.101, estimator.getState(0.01));

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.101, estimator.getState(0.01));
        verify(0.038, 0.101, estimator.getState(0.02));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.
        positions = position01;
        estimator.update(0.04);

        // LocalizerEstimate estimate14 = estimator.getState(0.00);
        verify(0.000, 0.1, estimator.getState(0.00));
        // LocalizerEstimate estimate15 = estimator.getState(0.02);
        verify(0.038, 0.102, estimator.getState(0.02));
        // LocalizerEstimate estimate16 = estimator.getState(0.04);
        verify(0.138, 0.102, estimator.getState(0.04));

        // here's the delayed update from above, which moves the estimate to 0.305 and
        // then the odometry is applied on top of that, yielding 0.405.
        vision = List.of(new VisionMeasurement(0.015, noisyMeasurement));
        estimator.update(0.04);
        vision = List.of();

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.077, 0.103, estimator.getState(0.015));
        // odometry thinks no motion at 0.02 so repeat the vision estimate here
        verify(0.077, 0.103, estimator.getState(0.02));
        // odometry of 0.1 + the vision estimate from 0.02.
        verify(0.177, 0.104, estimator.getState(0.04));

        // wheels are in the same position as the previous iteration,
        positions = position01;
        vision = List.of();
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.077, 0.103, estimator.getState(0.02));
        verify(0.177, 0.104, estimator.getState(0.04));
        verify(0.177, 0.104, estimator.getState(0.06));
        verify(0.177, 0.104, estimator.getState(0.08));

        // a little earlier than the previous estimate does nothing

        vision = List.of(new VisionMeasurement(0.014, noisyMeasurement));
        estimator.update(0.04);
        vision = List.of();

        verify(0.000, 0.1, estimator.getState(0.00));
        // notices the vision input a bit earlier
        verify(0.107, 0.104, estimator.getState(0.014));
        // but doesn't change this estimate since it's the same, and we're not moving,
        // we don't replay vision input
        // it would be better if two vision estimates pulled harder than one,
        // even if they come in out-of-order.
        verify(0.107, 0.104, estimator.getState(0.015));
        verify(0.107, 0.104, estimator.getState(0.02));
        verify(0.207, 0.105, estimator.getState(0.04));
        verify(0.207, 0.105, estimator.getState(0.06));

        // a little later than the previous estimate works normally.

        vision = List.of(new VisionMeasurement(0.016, noisyMeasurement));
        estimator.update(0.06);
        vision = List.of();

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.107, 0.104, estimator.getState(0.014));
        verify(0.107, 0.104, estimator.getState(0.015));
        // drag the pose towards the vision estimate a bit.
        verify(0.144, 0.105, estimator.getState(0.016));
        verify(0.144, 0.105, estimator.getState(0.02));
        verify(0.244, 0.106, estimator.getState(0.04));
        verify(0.244, 0.106, estimator.getState(0.06));

        // wheels not moving -> no change,

        positions = position01;
        vision = List.of();
        estimator.update(0.08);
        vision = List.of();

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.144, 0.105, estimator.getState(0.02));
        verify(0.244, 0.106, estimator.getState(0.04));
        verify(0.244, 0.106, estimator.getState(0.06));
        verify(0.244, 0.106, estimator.getState(0.08));
    }

    @Test
    void minorWeirdness() {
        // weirdness with out-of-order vision updates

        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        final IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        // initial pose = 0
        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.000, 0.1, estimator.getState(0.02));
        verify(0.000, 0.1, estimator.getState(0.04));
        verify(0.000, 0.1, estimator.getState(0.06));
        verify(0.000, 0.1, estimator.getState(0.08));

        // now vision says we're one meter away, so pose goes towards that
        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.01, noisyMeasurement));
        estimator.update(0.01);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.038, 0.102, estimator.getState(0.04));
        verify(0.038, 0.102, estimator.getState(0.06));
        verify(0.038, 0.102, estimator.getState(0.08));

        // if we had added this vision measurement here, it would have pulled the
        // estimate further
        // poseEstimator.addVisionMeasurement(visionRobotPoseMeters, 0.015);
        // verify(0.305, poseEstimator.apply());

        // wheels haven't moved, so the "odometry opinion" should be zero
        // but it's not, it's applied relative to the vision update, so there's no
        // change.

        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.038, 0.102, estimator.getState(0.04));
        verify(0.038, 0.102, estimator.getState(0.06));
        verify(0.038, 0.102, estimator.getState(0.08));

        // wheels have moved 0.1m in +x, at t=0.04.
        // the "odometry opinion" should be 0.1 since the last odometry estimate was
        // 0, but instead odometry is applied relative to the latest estimate, which
        // was based on vision. so the actual odometry stddev is like *zero*.

        positions = position01;
        vision = List.of();
        estimator.update(0.04);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.138, 0.102, estimator.getState(0.04));
        verify(0.138, 0.102, estimator.getState(0.06));
        verify(0.138, 0.102, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.015, noisyMeasurement));
        estimator.update(0.04);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.077, 0.103, estimator.getState(0.02));
        verify(0.177, 0.103, estimator.getState(0.04));
        verify(0.177, 0.103, estimator.getState(0.06));
        verify(0.177, 0.103, estimator.getState(0.08));

        // wheels are in the same position as the previous iteration

        positions = position01;
        vision = List.of();
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.077, 0.103, estimator.getState(0.02));
        verify(0.177, 0.103, estimator.getState(0.04));
        verify(0.177, 0.103, estimator.getState(0.06));
        verify(0.177, 0.103, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.014, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.107, 0.104, estimator.getState(0.02));
        verify(0.207, 0.104, estimator.getState(0.04));
        verify(0.207, 0.104, estimator.getState(0.06));
        verify(0.207, 0.104, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.016, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.144, 0.105, estimator.getState(0.02));
        verify(0.244, 0.105, estimator.getState(0.04));
        verify(0.244, 0.105, estimator.getState(0.06));
        verify(0.244, 0.105, estimator.getState(0.08));

        // wheels not moving -> no change

        positions = position01;
        vision = List.of();
        estimator.update(0.08);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.144, 0.105, estimator.getState(0.02));
        verify(0.244, 0.105, estimator.getState(0.04));
        verify(0.244, 0.105, estimator.getState(0.06));
        verify(0.244, 0.105, estimator.getState(0.08));
    }

    @Test
    void test0105() {
        // this is the current (post-comp 2024) base case.
        // within a few frames, the estimate converges on the vision input.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.000, 0.1, estimator.getState(0.02));
        verify(0.000, 0.1, estimator.getState(0.04));
        verify(0.000, 0.1, estimator.getState(0.06));
        verify(0.000, 0.1, estimator.getState(0.08));

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.02, noisyMeasurement));
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.038, 0.102, estimator.getState(0.04));
        verify(0.038, 0.102, estimator.getState(0.06));
        verify(0.038, 0.102, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.04, noisyMeasurement));
        estimator.update(0.04);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.077, 0.103, estimator.getState(0.04));
        verify(0.077, 0.103, estimator.getState(0.06));
        verify(0.077, 0.103, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.06, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.038, 0.102, estimator.getState(0.02));
        verify(0.077, 0.103, estimator.getState(0.04));
        verify(0.114, 0.104, estimator.getState(0.06));
        verify(0.114, 0.104, estimator.getState(0.08));
    }

    @Test
    void test0110() {
        // double vision stdev (r) -> slower convergence
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(1.0, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.1, 0.1)));

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.000, 0.1, estimator.getState(0.02));
        verify(0.000, 0.1, estimator.getState(0.04));
        verify(0.000, 0.1, estimator.getState(0.06));
        verify(0.000, 0.1, estimator.getState(0.08));

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.02, noisyMeasurement));
        estimator.update(0.02);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.010, 0.100, estimator.getState(0.02));
        verify(0.010, 0.100, estimator.getState(0.04));
        verify(0.010, 0.100, estimator.getState(0.06));
        verify(0.010, 0.100, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.04, noisyMeasurement));
        estimator.update(0.04);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.010, 0.100, estimator.getState(0.02));
        verify(0.020, 0.100, estimator.getState(0.04));
        verify(0.020, 0.100, estimator.getState(0.06));
        verify(0.020, 0.100, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.06, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.1, estimator.getState(0.00));
        verify(0.010, 0.100, estimator.getState(0.02));
        verify(0.020, 0.100, estimator.getState(0.04));
        verify(0.029, 0.101, estimator.getState(0.06));
        verify(0.029, 0.101, estimator.getState(0.08));
    }

    @Test
    void test00505() {
        // half odo stdev (q) -> slower convergence
        // the K is q/(q+qr) so it's q compared to r that matters.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.5, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.05, 0.05)));

        verify(0.000, 0.05, estimator.getState(0.00));
        verify(0.000, 0.05, estimator.getState(0.02));
        verify(0.000, 0.05, estimator.getState(0.04));
        verify(0.000, 0.05, estimator.getState(0.06));
        verify(0.000, 0.05, estimator.getState(0.08));

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.02, noisyMeasurement));
        estimator.update(0.02);

        verify(0.000, 0.05, estimator.getState(0.00));
        verify(0.010, 0.052, estimator.getState(0.02));
        verify(0.010, 0.052, estimator.getState(0.04));
        verify(0.010, 0.052, estimator.getState(0.06));
        verify(0.010, 0.052, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.04, noisyMeasurement));
        estimator.update(0.04);

        verify(0.000, 0.05, estimator.getState(0.00));
        verify(0.010, 0.052, estimator.getState(0.02));
        verify(0.020, 0.053, estimator.getState(0.04));
        verify(0.020, 0.053, estimator.getState(0.06));
        verify(0.020, 0.053, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.06, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.05, estimator.getState(0.00));
        verify(0.010, 0.052, estimator.getState(0.02));
        verify(0.020, 0.053, estimator.getState(0.04));
        verify(0.031, 0.055, estimator.getState(0.06));
        verify(0.031, 0.055, estimator.getState(0.08));
    }

    @Test
    void reasonable() {
        // stdev that actually make sense
        // actual odometry error is very low
        // measured camera error is something under 10 cm
        // these yield much slower convergence, maybe too slow? try and see.
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

        IsotropicNoiseSE2 visionMeasurementStdDevs = IsotropicNoiseSE2.fromStdDev(0.1, Double.MAX_VALUE);

        OdometryEstimator odometryEstimator = new OdometryEstimator(
                log, kinodynamics, 4e-4, 1e-5,
                false, true);

        NudgingVisionEstimator visionEstimator = new NudgingVisionEstimator(log);

        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();

        FusedEstimator estimator = new FusedEstimator(
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                odometryEstimator,
                visionEstimator);
        estimator.reset(
                0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.fromStdDev(0.01, 0.01)));

        verify(0.000, 0.01, estimator.getState(0.00));
        verify(0.000, 0.01, estimator.getState(0.02));
        verify(0.000, 0.01, estimator.getState(0.04));
        verify(0.000, 0.01, estimator.getState(0.06));
        verify(0.000, 0.01, estimator.getState(0.08));

        NoisyPose2d noisyMeasurement = new NoisyPose2d(visionPose, visionMeasurementStdDevs);

        vision = List.of(new VisionMeasurement(0.02, noisyMeasurement));
        estimator.update(0.02);

        verify(0.000, 0.01, estimator.getState(0.00));
        verify(0.010, 0.017, estimator.getState(0.02));
        verify(0.010, 0.017, estimator.getState(0.04));
        verify(0.010, 0.017, estimator.getState(0.06));
        verify(0.010, 0.017, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.04, noisyMeasurement));
        estimator.update(0.04);

        verify(0.000, 0.01, estimator.getState(0.00));
        verify(0.010, 0.017, estimator.getState(0.02));
        verify(0.038, 0.029, estimator.getState(0.04));
        verify(0.038, 0.029, estimator.getState(0.06));
        verify(0.038, 0.029, estimator.getState(0.08));

        vision = List.of(new VisionMeasurement(0.06, noisyMeasurement));
        estimator.update(0.06);

        verify(0.000, 0.01, estimator.getState(0.00));
        verify(0.010, 0.017, estimator.getState(0.02));
        verify(0.038, 0.029, estimator.getState(0.04));
        verify(0.112, 0.046, estimator.getState(0.06));
        verify(0.112, 0.046, estimator.getState(0.08));
    }

}
