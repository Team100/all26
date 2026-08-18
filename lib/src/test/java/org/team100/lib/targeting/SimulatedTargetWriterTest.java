package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Timeless because the clock is used to decide to ignore (stale) input. */
public class SimulatedTargetWriterTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(
            new TestPrimitiveLogger());

    @Test
    void testOne() throws InterruptedException {
        if (DEBUG)
            System.out.println("testOne");
        NetworkTableInstance.getDefault().startServer();
        Thread.sleep(100);

        stepTime();

        ModelSE2 p = new ModelSE2();
        Targets reader = new Targets(logger, logger, 100, x -> p);
        Thread.sleep(100);
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                logger,
                List.of(Camera.TEST4),
                x -> p,
                new Translation2d[] {
                        new Translation2d(1, 0) });

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        writer.update();

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        reader.update();

        assertEquals(1, reader.getTargets().size());
        Translation2d target = reader.getTargets().get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

        writer.close();

    }

}
