package org.team100.lib.localization.gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.ImmutableSwerveHistory;
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
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.util.Nat;

import gtsam.Key;
import gtsam.Matrix;
import gtsam.PlanarProjectionFactor1;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector2;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class GtsamStateEstimatorTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private Rotation2d yaw;
    private SwerveModulePositions positions;
    private List<Corners> vision;

    @Test
    void test0() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        // this update does nothing since the estimator already contains a value for 0.
        ImmutableSwerveHistory h = estimator.update(0.00);
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        estimator.dump();
    }

    @Test
    void test0a() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        // this update does nothing since the estimator already contains a value for 0.
        ImmutableSwerveHistory h = estimator.update(0.01);
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        estimator.dump();
    }

    @Test
    void test1() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        // explicit reset
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        estimator.dump();
    }

    @Test
    void test2() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        // explicit reset
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        // verify the state we just set
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());

        // update with motion
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)));
        h = estimator.update(0.02);

        // verify the first state hasn't changed
        state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());

        // verify the new state, note 0.02 movement in 0.02 sec => 1 m/s
        state = h.get(0.02);
        TestUtil.verify(new Pose2d(0.02, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(1, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        estimator.dump();
    }

    @Test
    void test3() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());

        // vision lags more than one update period, so do one odometry update first.
        h = estimator.update(0.02);
        state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        state = h.get(0.02);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());

        // add vision input consistent with pose at (0,0,0).
        Translation3d landmark = new Translation3d(1, 0, 0);
        Translation2d measurement = new Translation2d(200, 200);
        Intrinsic intrinsic = new Intrinsic(new double[][] { //
                { 200.0, 0.0, 200.0 }, //
                { 0.0, 200.0, 200.0 }, //
                { 0.0, 0.0, 1.0 } });
        // this is a WPI-land offset
        Transform3d offset = new Transform3d();
        vision = List.of(new Corners(
                0.01,
                landmark,
                measurement,
                offset,
                intrinsic,
                Distortion.DEFAULT));
        // this update should collect the vision input and interpolate.
        h = estimator.update(0.04);
        state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        state = h.get(0.01);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        state = h.get(0.02);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());
        state = h.get(0.04);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(0.1, 0.1), state.noise());

        estimator.dump();

        Map<Double, NoisyStateSE2> all = estimator.all();
        System.out.printf("all size %d\n", all.size());
        for (Map.Entry<Double, NoisyStateSE2> s : all.entrySet()) {
            System.out.printf("%f %s\n", s.getKey(), s.getValue());
        }
    }

    @Test
    void test4() {
        yaw = Rotation2d.kZero;
        positions = SwerveModulePositions.ZERO;
        vision = List.of();
        GtsamStateEstimator estimator = new GtsamStateEstimator(
                log,
                0,
                () -> yaw,
                () -> positions,
                () -> vision,
                kinodynamics.getKinematics(),
                1e-4,
                3e-5);
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 10), state.noise());

        // vision lags more than one update period, so do one odometry update first.
        h = estimator.update(0.02);
        state = h.get(0.00);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 10), state.noise());
        state = h.get(0.02);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 10), state.noise());

        // add vision input that is not consistent with pose at (0,0,0).
        Translation3d landmark = new Translation3d(1, 0, 0);
        // note the "x" dimension; this indicates the robot is +y from the origin
        Translation2d measurement = new Translation2d(300, 200);
        Intrinsic intrinsic = new Intrinsic(new double[][] { //
                { 200.0, 0.0, 200.0 }, //
                { 0.0, 200.0, 200.0 }, //
                { 0.0, 0.0, 1.0 } });
        // in WPI-land there is no offset, the CAM offset is applied later.
        Transform3d offset = new Transform3d();
        vision = List.of(new Corners(
                0.01,
                landmark,
                measurement,
                offset,
                intrinsic,
                Distortion.DEFAULT));
        h = estimator.update(0.04);
        state = h.get(0.00);
        TestUtil.verify(new Pose2d(0.054, 0.227, new Rotation2d(0.227)), state.pose());
        TestUtil.verify(new VelocitySE2(0, 0, 0), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(8.550, 7.178), state.noise());
        state = h.get(0.01);
        TestUtil.verify(new Pose2d(0.055, 0.228, new Rotation2d(0.227)), state.pose());
        TestUtil.verify(new VelocitySE2(0.000, 0.000, 0.000), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(8.550, 7.178), state.noise());
        state = h.get(0.02);
        TestUtil.verify(new Pose2d(0.055, 0.228, new Rotation2d(0.227)), state.pose());
        TestUtil.verify(new VelocitySE2(0.000, 0.000, 0.000), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(8.550, 7.178), state.noise());
        state = h.get(0.04);
        TestUtil.verify(new Pose2d(0.055, 0.228, new Rotation2d(0.227)), state.pose());
        TestUtil.verify(new VelocitySE2(0.000, 0.000, 0.000), state.velocity());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(8.550, 7.178), state.noise());
        estimator.dump();

        Map<Double, NoisyStateSE2> all = estimator.all();
        System.out.printf("all size %d\n", all.size());
        for (Map.Entry<Double, NoisyStateSE2> s : all.entrySet()) {
            System.out.printf("%f %s\n", s.getKey(), s.getValue());
        }
    }

    @Test
    void testVision() {
        Key x = Key.X(0);
        Translation3d landmark = new Translation3d(1, 0, 0);
        // note vision signal here does not match the pose
        Translation2d measurement = new Translation2d(300, 200);
        // this is a GTSAM offset
        Transform3d offset = new Transform3d(
                new Translation3d(),
                new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0)));
        Intrinsic intrinsic = new Intrinsic(new double[][] { //
                { 200.0, 0.0, 200.0 }, //
                { 0.0, 200.0, 200.0 }, //
                { 0.0, 0.0, 1.0 } });
        shared_ptr<Diagonal> PX_NOISE = Diagonal.Sigmas(new Vector2(2, 2));

        shared_ptr<PlanarProjectionFactor1> ptr = PlanarProjectionFactor1.newPlanarProjectionFactor1(
                x,
                Geometry.toPoint3(landmark),
                Geometry.toPoint2(measurement),
                Geometry.toPose3(offset),
                Geometry.toCal3DS2(intrinsic, Distortion.DEFAULT),
                PX_NOISE);

        Pose2 pose = new Pose2();
        Matrix H = new Matrix();

        PlanarProjectionFactor1 factor = ptr.get();
        Vector2 err1 = factor.evaluateError(pose, H);
        // significant error
        assertEquals(-100, err1.at(0), 0.001);
        assertEquals(0, err1.at(1), 0.001);

        Values v = new Values();
        v.insert(x, new Pose2());
        double err2 = factor.error(v);
        // significant error
        assertEquals(1250, err2, 0.001);
    }

}
