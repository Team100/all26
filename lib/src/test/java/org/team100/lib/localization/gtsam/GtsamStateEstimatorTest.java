package org.team100.lib.localization.gtsam;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Distortion;
import org.team100.lib.camera.Intrinsic;
import org.team100.lib.geometry.se2.VelocitySE2;
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
        ImmutableSwerveHistory h = estimator.update(0.00);
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new StateSE2(), state.state());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 6), state.noise());
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
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new StateSE2(), state.state());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 6), state.noise());
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
        System.out.println("#### reset at zero");
        estimator.reset(0,
                new NoisyStateSE2(
                        new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                        IsotropicNoiseSE2.high()));
        // verify the state we just set
        ImmutableSwerveHistory h = estimator.immutableCopy();
        NoisyStateSE2 state = h.get(0.00);
        TestUtil.verify(new StateSE2(), state.state());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 6), state.noise());

        System.out.println("#### update at 0.02");
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)),
                new SwerveModulePosition100(0.02, Optional.of(Rotation2d.kZero)));
        h = estimator.update(0.02);

        // verify the first state hasn't changed
        state = h.get(0.00);
        TestUtil.verify(new StateSE2(), state.state());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 6), state.noise());

        // verify the new state, note 0.02 movement in 0.02 sec => 1 m/s
        state = h.get(0.02);
        TestUtil.verify(
                new StateSE2(
                        new Pose2d(0.02, 0, Rotation2d.kZero),
                        new VelocitySE2(1, 0, 0)),
                state.state());
        TestUtil.verify(IsotropicNoiseSE2.fromStdDev(10, 6), state.noise());
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
        Translation3d landmark = new Translation3d(1, 0, 0);
        Translation2d measurement = new Translation2d(200, 200);
        Intrinsic intrinsic = new Intrinsic(new double[][] { //
                { 200.0, 0.0, 200.0 }, //
                { 0.0, 200.0, 200.0 }, //
                { 0.0, 0.0, 1.0 } });
        Transform3d offset = new Transform3d(
                new Translation3d(),
                new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                        0, 0, 1, //
                        -1, 0, 0, //
                        0, -1, 0)));
        vision = List.of(new Corners(
                0.01,
                landmark,
                measurement,
                offset,
                intrinsic,
                Distortion.DEFAULT));
        estimator.update(0.02);
        estimator.dump();
        Map<Double, NoisyStateSE2> all = estimator.all();
        System.out.printf("all size %d\n", all.size());
        for (Map.Entry<Double, NoisyStateSE2> s : all.entrySet()) {
            System.out.printf("%f %s\n", s.getKey(), s.getValue());
        }
    }

}
