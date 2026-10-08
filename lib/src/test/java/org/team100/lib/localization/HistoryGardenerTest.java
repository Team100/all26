package org.team100.lib.localization;

import java.util.function.UnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.sensor.gyro.MockGyro;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class HistoryGardenerTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private SwerveModulePositions positions;

    @Test
    void test0() {
        MockGyro gyro = new MockGyro();
        positions = SwerveModulePositions.kZero();
        SwerveHistory history = new SwerveHistory(
                log,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positions,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);
        OdometryUpdater ou = new OdometryUpdater(
                log, kinodynamics, gyro, history,
                () -> positions, UnaryOperator.identity(), true);

        NudgingVisionUpdater vu = new NudgingVisionUpdater(log, history, ou);

        HistoryGardener gardener = new HistoryGardener(history, ou, vu);

        gardener.putOdometry(0.00, Rotation2d.kZero, positions);
        gardener.putOdometry(0.02, Rotation2d.kZero, positions);
        gardener.putVision(0.01, new NoisyPose2d(new Pose2d(), IsotropicNoiseSE2.fromStdDev(1, 1)));
        gardener.sweep();

        history.dump();
    }

}
