package org.team100.lib.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motor.wpi.MotorController100;

class MotorController100Test {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTurning() {
        MockMotorController mc = new MockMotorController();
        MotorController100 m = new MotorController100(logger, mc);
        m.setVelocity(0, 0);
        assertEquals(0, mc.speed, DELTA);
        m.setVelocity(1, 0);
        assertEquals(0.0016, mc.speed, DELTA);
    }
}
