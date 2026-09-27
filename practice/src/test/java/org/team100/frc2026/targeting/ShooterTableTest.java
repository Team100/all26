package org.team100.frc2026.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.targeting.FiringParameters;

public class ShooterTableTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        ShooterTable t = new ShooterTable(
                List.of(
                        new FiringParameters(2, 3, 4, 5),
                        new FiringParameters(4, 5, 6, 7)));
        assertTrue(t.angle(1).isEmpty());
        assertEquals(4, t.angle(2).getAsDouble(), DELTA);
        assertEquals(6, t.angle(4).getAsDouble(), DELTA);
        assertTrue(t.angle(6).isEmpty());
    }

    @Test
    void test1() {
        ShooterTable t = new ShooterTable(
                List.of(
                        new FiringParameters(2, 3, 4, 5),
                        new FiringParameters(4, 5, 6, 7)));
        assertTrue(t.tof(1).isEmpty());
        assertEquals(5, t.tof(2).getAsDouble(), DELTA);
        assertEquals(7, t.tof(4).getAsDouble(), DELTA);
        assertTrue(t.tof(6).isEmpty());
    }

    @Test
    void test2() {
        ShooterTable t = new ShooterTable(
                List.of(
                        new FiringParameters(2, 3, 4, 5),
                        new FiringParameters(4, 5, 6, 7)));
        assertTrue(t.forRange(1).isEmpty());
        assertEquals(5, t.forRange(2).get().tof(), DELTA);
        assertEquals(7, t.forRange(4).get().tof(), DELTA);
        assertTrue(t.forRange(6).isEmpty());

    }
}
