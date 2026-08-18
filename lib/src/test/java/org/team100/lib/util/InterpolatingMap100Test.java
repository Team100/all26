package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingMap100Test {
    private static final double DELTA = 1e-12;

    /** Off the end -> null */
    @Test
    void test0() {
        InterpolatingMap100<Double, Double> m = new InterpolatingMap100<>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());
        m.put(0.0, 0.0);
        m.put(1.0, 1.0);
        assertNull(m.get(-1.0));
        assertEquals(0.0, m.get(0.0), DELTA);
        assertEquals(0.5, m.get(0.5), DELTA);
        assertEquals(1.0, m.get(1.0), DELTA);
        assertNull(m.get(2.0));
    }

    /** For comparison. */
    @Test
    void testWPI() {
        InterpolatingTreeMap<Double, Double> m = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());
        m.put(0.0, 0.0);
        m.put(1.0, 1.0);
        assertEquals(0.0, m.get(-1.0), DELTA);
        assertEquals(0.0, m.get(0.0), DELTA);
        assertEquals(0.5, m.get(0.5), DELTA);
        assertEquals(1.0, m.get(1.0), DELTA);
        assertEquals(1.0, m.get(2.0), DELTA);
    }

}
