package org.team100.frc2026.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.targeting.FiringParameters;

import edu.wpi.first.math.geometry.Translation2d;

public class TargeterTest {
    @Test
    void testShoot() {
        Targeter targeter = new Targeter(() -> new Translation2d(1, 1));
        // impossible (hits the hub)
        assertTrue(targeter.forRange(0).isEmpty());
        // impossible (beyond the wall)
        assertTrue(targeter.forRange(10).isEmpty());

        // in the alliance zone
        FiringParameters fp = targeter.forRange(3).get();
        assertEquals(0.151, fp.elevation(), 0.001);
        assertEquals(14.019, fp.speed(), 0.001);
        assertEquals(0.846, fp.tof(), 0.001);
    }

    @Test
    void testLob() {
        // lob lookup never fails.
        Targeter targeter = new Targeter(() -> new Translation2d(8, 0));
        FiringParameters fp = targeter.forRange(6).get();
        assertEquals(0.25, fp.elevation(), 0.001);
        assertEquals(17, fp.speed(), 0.001);
        assertEquals(1.268, fp.tof(), 0.001);
    }

}
