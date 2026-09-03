package org.team100.lib.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ControlUtilTest {
    private static final double DELTA = 0.001;

    @Test
    void testExpo() {
        assertEquals(0.3125, ControlUtil.expo(0.5, 0.5), DELTA);
    }

    @Test
    void testDeadband() {
        assertEquals(0, ControlUtil.deadband(0.1, 0.2, 1.0), DELTA);

    }

    @Test
    void testClamp() {
        assertEquals(0.5, ControlUtil.clamp(2, 0.5), DELTA);
    }

    @Test
    void testVelocity() {
        // squashes the input to a circle.  This should only be used
        // for controls with circular envelopes.
        Velocity v = ControlUtil.velocity(
                () -> 1.0,
                () -> 1.0,
                () -> 0.0,
                0,
                0);
        assertEquals(-0.707, v.x(), DELTA);
        assertEquals(-0.707, v.y(), DELTA);
    }

}
