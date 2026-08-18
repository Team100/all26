package org.team100.lib.visualization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;

public class BallR2Test {
    private static final boolean DEBUG = false;

    ModelSE2 robot = new ModelSE2();
    Rotation2d azimuth = new Rotation2d();

    /** 2d ball proceeds forever. */
    @Test
    void testBall() {
        LoggerFactory field = new TestLoggerFactory(new TestPrimitiveLogger());
        BallR2 b = new BallR2(field, () -> robot, () -> azimuth, () -> 1);
        assertNull(b.m_location);
        assertNull(b.m_velocity);
        b.launch();
        assertEquals(0, b.m_location.getX());
        assertEquals(0, b.m_location.getY());
        assertEquals(1, b.m_velocity.x());
        assertEquals(0, b.m_velocity.y());
        for (double t = 0; t < 10; t += 0.02) {
            b.fly();
            if (DEBUG)
                System.out.printf("%f %f %f %f\n",
                        b.m_location.getX(),
                        b.m_location.getY(),
                        b.m_velocity.x(),
                        b.m_velocity.y());
        }
    }

}
