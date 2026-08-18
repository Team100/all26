package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;

/**
 * Timeless because the clock is used to decide to ignore (stale) input.
 */
public class TargetsTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(
            new TestPrimitiveLogger());

    @Test
    void testTargets() throws InterruptedException {
        NetworkTableInstance.getDefault().startServer();
        Thread.sleep(200);
        stepTime();

        ModelSE2 p = new ModelSE2();
        Targets t = new Targets(logger, logger, 100, (x) -> p);
        t.update();
        assertTrue(t.getTargets().isEmpty());
        // send some blips

        // client instance
        NetworkTableInstance inst = NetworkTableInstance.create();
        inst.setServer("localhost");
        inst.startClient4("tag_finder24");

        // wait for the NT thread
        Thread.sleep(200);
        assertTrue(inst.isConnected());

        // test4 camera offset is 0,0,1, without rotation
        StructArrayTopic<Target> topic = inst.getStructArrayTopic(
                "objectVision/test4/targets", Target.struct);
        StructArrayPublisher<Target> pub = topic.publish();
        stepTime();
        // tilt down 45
        pub.set(new Target[] { new Target(0, new Rotation3d(0, Math.PI / 4, 0)) });

        // wait for NT rate-limiting
        Thread.sleep(200);
        inst.flush();
        stepTime();
        t.update();
        assertEquals(1, t.getTargets().size());
        Translation2d target = t.getTargets().get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

        inst.close();
    }

    @Test
    void testTranslations() throws InterruptedException {
        NetworkTableInstance.getDefault().startServer();
        Thread.sleep(100);
        stepTime();

        ModelSE2 p = new ModelSE2();
        Targets reader = new Targets(logger, logger, 100, (x) -> p);
        Thread.sleep(200);
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                logger,
                List.of(Camera.TEST4),
                x -> p,
                new Translation2d[] { new Translation2d(1, 0) });

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        writer.update();

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        assertEquals(1, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleCameras() throws InterruptedException {
        NetworkTableInstance.getDefault().startServer();
        Thread.sleep(100);
        stepTime();

        ModelSE2 p = new ModelSE2();
        Targets reader = new Targets(logger, logger, 100, (x) -> p);
        Thread.sleep(100);
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                logger,
                List.of(Camera.TEST4, Camera.TEST5),
                x -> p,
                new Translation2d[] { new Translation2d(1, 0) });

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        writer.update();

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        // both cameras see the sme target
        assertEquals(1, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleTargets() throws InterruptedException {
        if (DEBUG)
            System.out.println("testMultipleTargets");
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
                        new Translation2d(1, 0),
                        new Translation2d(2, 0) });

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        writer.update();

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        assertEquals(2, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

    @Test
    void testMultipleTargetsAndCameras() throws InterruptedException {
        if (DEBUG)
            System.out.println("testMultipleTargetsAndCameras");
        NetworkTableInstance.getDefault().startServer();
        Thread.sleep(100);
        stepTime();

        ModelSE2 p = new ModelSE2();
        Targets reader = new Targets(logger, logger, 100, (x) -> p);
        Thread.sleep(50);
        SimulatedTargetWriter writer = new SimulatedTargetWriter(
                logger,
                List.of(Camera.TEST4, Camera.TEST5),
                x -> p,
                new Translation2d[] {
                        new Translation2d(1, 0),
                        new Translation2d(2, 0) });

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        writer.update();

        // wait for NT rate-limiting
        Thread.sleep(100);

        stepTime();
        reader.update();

        List<Translation2d> allTargets = reader.getTargets();
        // multi-camera views of the same target are coalesced
        assertEquals(2, allTargets.size());

        Optional<Translation2d> tt = reader.getClosestTarget();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }
}
