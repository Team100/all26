package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.DoubleFunction;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;

public class RelationTest {
    private static final double DELTA = 1e-12;

    public  record Tuple(double x, double y) {
        public static Tuple interpolate(Tuple a, Tuple b, double t) {
            return new Tuple(
                    MathUtil.interpolate(a.x, b.x, t),
                    MathUtil.interpolate(a.y, b.y, t));
        }
    }

    @Test
    void testMap() {
        Relation<Tuple> f = new Relation<>();
        f.add(new Tuple(0.0, 0.0));
        f.add(new Tuple(0.5, 5.0));
        f.add(new Tuple(1.0, 10.0));

        InterpolatingMap100<Double, Tuple> m = f.map(
                v -> v.x(),
                Tuple::interpolate);
        assertNull(m.get(-1.0));
        assertEquals(0.0, m.get(0.0).y(), DELTA);
        assertEquals(2.5, m.get(0.25).y(), DELTA);
        assertEquals(5.0, m.get(0.5).y(), DELTA);
        assertEquals(7.5, m.get(0.75).y(), DELTA);
        assertEquals(10.0, m.get(1.0).y(), DELTA);
        assertNull(m.get(2.0));
    }

    @Test
    void testFn() {
        Relation<Tuple> f = new Relation<>();
        f.add(new Tuple(0.0, 0.0));
        f.add(new Tuple(0.5, 5.0));
        f.add(new Tuple(1.0, 10.0));
        DoubleFunction<Double> m = f.function(
                v -> v.x(),
                v -> v.y());
        assertNull(m.apply(-1.0));
        assertEquals(0.0, m.apply(0.0), DELTA);
        assertEquals(2.5, m.apply(0.25), DELTA);
        assertEquals(5.0, m.apply(0.5), DELTA);
        assertEquals(7.5, m.apply(0.75), DELTA);
        assertEquals(10.0, m.apply(1.0), DELTA);
        assertNull(m.apply(2.0));
    }

    @Test
    void testInvMap() {
        Relation<Tuple> f = new Relation<>();
        f.add(new Tuple(0.0, 0.0));
        f.add(new Tuple(0.5, 5.0));
        f.add(new Tuple(1.0, 10.0));

        InterpolatingMap100<Double, Tuple> m = f.map(
                v -> v.y(),
                Tuple::interpolate);
        assertNull(m.get(-10.0));
        assertEquals(0.0, m.get(0.0).x(), DELTA);
        assertEquals(0.25, m.get(2.5).x(), DELTA);
        assertEquals(0.5, m.get(5.0).x(), DELTA);
        assertEquals(0.75, m.get(7.5).x(), DELTA);
        assertEquals(1.0, m.get(10.0).x(), DELTA);
        assertNull(m.get(20.0));
    }

}
