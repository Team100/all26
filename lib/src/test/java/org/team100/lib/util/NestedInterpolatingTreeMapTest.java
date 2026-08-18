package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class NestedInterpolatingTreeMapTest {
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        NestedInterpolatingTreeMap<Double, Double> map = new NestedInterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());
        // z=x+y
        map.put(0.0, 0.0, 0.0);
        map.put(1.0, 0.0, 1.0);
        map.put(0.0, 1.0, 1.0);
        map.put(1.0, 1.0, 2.0);
        // verify z = x+y
        assertEquals(0.25, map.get(0.00, 0.25), DELTA);
        assertEquals(0.50, map.get(0.00, 0.50), DELTA);
        assertEquals(0.75, map.get(0.00, 0.75), DELTA);
        assertEquals(0.50, map.get(0.25, 0.25), DELTA);
        assertEquals(0.75, map.get(0.25, 0.50), DELTA);
        assertEquals(1.00, map.get(0.25, 0.75), DELTA);
        assertEquals(0.75, map.get(0.50, 0.25), DELTA);
        assertEquals(1.00, map.get(0.50, 0.50), DELTA);
        assertEquals(1.25, map.get(0.50, 0.75), DELTA);
        assertEquals(1.00, map.get(0.75, 0.25), DELTA);
        assertEquals(1.25, map.get(0.75, 0.50), DELTA);
        assertEquals(1.50, map.get(0.75, 0.75), DELTA);
        assertEquals(1.25, map.get(1.00, 0.25), DELTA);
        assertEquals(1.50, map.get(1.00, 0.50), DELTA);
        assertEquals(1.75, map.get(1.00, 0.75), DELTA);
        // does not extrapolate
        assertEquals(1.00, map.get(2.00, 0.00), DELTA);
        assertEquals(0.00, map.get(0.00, -1.00), DELTA);
        assertEquals(2.00, map.get(10.00, 10.00), DELTA);
    }

}
