package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.localization.fused.AprilTagCornerTranslator;
import org.team100.lib.localization.fused.VisionMeasurement;
import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

public class AprilTagCornerRobotLocalizerTest {
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void test0() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        AprilTagCornerTranslator translator = new AprilTagCornerTranslator(
                logger, layout, () -> Optional.of(Alliance.RED));
        List<VisionMeasurement> measurement = translator.convert(
                Camera.SIM0,
                new BlipWithCorners[] {
                        new BlipWithCorners(0,
                                4,
                                500, 600, 600, 600, 600, 500, 500, 500,
                                new Transform3d()) });
        assertEquals(1, measurement.size());
        VisionMeasurement m = measurement.get(0);
        assertEquals(0.0, m.timestamp(), DELTA);
        NoisyPose2d noisyMeasurement = m.noisyMeasurement();
        Pose2d pose = noisyMeasurement.pose();
        assertEquals(7.310, pose.getX(), DELTA);
        assertEquals(1.914, pose.getY(), DELTA);
    }
}
