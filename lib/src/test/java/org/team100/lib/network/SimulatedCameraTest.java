package org.team100.lib.network;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SimulatedCameraTest implements Timeless {
    private static final LoggerFactory log = new TestLoggerFactory(new TestPrimitiveLogger());

    Rotation2d testR = null;
    Rotation2d cameraRotation = null;

    private void acceptTag(Transform3d t, double timestamp) {
        cameraRotation = new Rotation2d(t.getRotation().getX());
    }

    @Test
    void test0() throws InterruptedException {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();
        Thread.sleep(500);
        stepTime();

        // Use a real camera.
        RawTags camera = new RawTags(log, this::acceptTag);

        // This provides input to the real camera
        SimulatedCamera sim = new SimulatedCamera((x) -> testR);

        // Input = 0.
        testR = new Rotation2d(0);
        sim.run();

        // wait for NT rate-limiting
        Thread.sleep(500);
        stepTime();

        camera.update();
        // Output = 0.
        assertEquals(0.000000, cameraRotation.getRadians(), 0.000001);

        // Input = pi/2.
        testR = new Rotation2d(Math.PI / 2);
        sim.run();

        // wait for NT rate-limiting
        Thread.sleep(500);
        stepTime();

        camera.update();
        // Output = pi/2.
        assertEquals(Math.PI / 2, cameraRotation.getRadians(), 0.000001);

    }

}
