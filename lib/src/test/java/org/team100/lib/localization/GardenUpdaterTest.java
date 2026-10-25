package org.team100.lib.localization;

import java.util.List;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GardenUpdaterTest {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forRealisticTest();

    private SwerveModulePositions positions;
    private VisionMeasurement vision;

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
        OdometryEstimator ou = new OdometryEstimator(
                log, kinodynamics, gyro.white_noise(),
                gyro.bias_noise(), history::lowerEntry,
                false, true);
        OdometryReplayer or = new OdometryReplayer(history, ou);
        NudgingVisionEstimator vu = new NudgingVisionEstimator(log, history::getRecord);

        HistoryGardener gardener = new HistoryGardener(history, ou, or, vu);

        vision = new VisionMeasurement(
                0.01, new NoisyPose2d(new Pose2d(), IsotropicNoiseSE2.fromStdDev(1, 1)));

        GardenUpdater gu = new GardenUpdater(
                gyro, () -> positions, () -> List.of(vision), gardener);

        gu.update(0.02);

        history.dump();
    }

}
