package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagCornerRobotLocalizerTest {
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        AprilTagCornerTranslator m_translator = new AprilTagCornerTranslator(
                logger, layout, () -> Optional.of(Alliance.Red));
        BlipWithCorners tag4 = new BlipWithCorners(0, 4, 500, 600, 600, 600, 600, 500, 500, 500, new Transform3d());
        BlipWithCorners[] tags = new BlipWithCorners[] { tag4 };
        Camera camera = Camera.SIM0;
        List<VisionMeasurement> measurement = new ArrayList<>();

        measurement.addAll(m_translator.convert(camera, tags));
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(7.310, m.noisyMeasurement().pose().getX(), DELTA);
        assertEquals(1.914, m.noisyMeasurement().pose().getY(), DELTA);
    }

    @Test
    void test1() throws IOException {
        // vision-only, watch the nudging
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);

        SwerveModulePosition100 p0 = new SwerveModulePosition100(0, Optional.of(Rotation2d.kZero));

        SwerveModulePositions positionZero = new SwerveModulePositions(p0, p0, p0, p0);

        SwerveHistory history = new SwerveHistory(
                logger,
                0.2,
                Rotation2d.kZero,
                VariableR1.fromVariance(0, 1),
                positionZero,
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                0);
        // history is never empty
        assertEquals(1, history.size());
        double lastKey = history.lastKey();
        assertEquals(0, lastKey, DELTA);
        SwerveState state = history.getRecord(lastKey);
        Pose2d p = state.state().pose();
        assertEquals(0, p.getX(), DELTA);
        assertEquals(0, p.getY(), DELTA);
        // high uncertainty
        assertEquals(10, state.noise().cartesian(), DELTA);

        // no replayer
        NudgingVisionEstimator visionUpdater = new NudgingVisionEstimator(
                logger, history::getRecord);
        AprilTagCornerTranslator m_translator = new AprilTagCornerTranslator(
                logger, layout, () -> Optional.of(Alliance.Red));

        Camera camera = Camera.SIM0;
        // watch the tag for 0.2 sec
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (double t = 0.02; t <= 0.2; t += 0.02) {
            measurements.addAll(m_translator.convert(camera, getTags(t)));
        }
        for (VisionMeasurement m : measurements) {
            SwerveState s = visionUpdater.estimate(m.timestamp(), m.noisyMeasurement());
            history.put(m.timestamp(), s);
        }
        assertEquals(11, history.size());
        // pose is most of the way but not all the way; the nudging is proportional to
        // the "innovation" i.e. difference between new estimate and history
        lastKey = history.lastKey();
        assertEquals(0.2, lastKey, DELTA);
        state = history.getRecord(lastKey);
        p = state.state().pose();
        assertEquals(7.157, p.getX(), DELTA);
        assertEquals(1.874, p.getY(), DELTA);
        // uncertainty is still pretty high because the tag is directly in front of the
        // camera, which is a very uncertain state.
        assertEquals(1.455, state.noise().cartesian(), DELTA);

        // watch the tag for 0.2 more sec
        measurements.clear();
        for (double t = 0.22; t <= 0.4; t += 0.02) {
            measurements.addAll(m_translator.convert(camera, getTags(t)));
        }
        for (VisionMeasurement m : measurements) {
            SwerveState s = visionUpdater.estimate(m.timestamp(), m.noisyMeasurement());
            history.put(m.timestamp(), s);
        }
        assertEquals(10, history.size());
        lastKey = history.lastKey();
        assertEquals(0.38, lastKey, DELTA);
        // pose is closer now but still not that close, because of the high uncertainty
        // the nudger doesn't nudge very hard.
        state = history.getRecord(lastKey);
        p = state.state().pose();
        assertEquals(7.224, p.getX(), DELTA);
        assertEquals(1.891, p.getY(), DELTA);
        assertEquals(1.060, state.noise().cartesian(), DELTA);
    }

    private BlipWithCorners[] getTags(double timestampsec) {
        long t = (long) (timestampsec * 1e6);
        BlipWithCorners tag4 = new BlipWithCorners(t, 4, 500, 600, 600, 600, 600, 500, 500, 500, new Transform3d());
        BlipWithCorners[] tags = new BlipWithCorners[] { tag4 };
        return tags;
    }
}
