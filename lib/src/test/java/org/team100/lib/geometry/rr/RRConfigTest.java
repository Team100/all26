package org.team100.lib.geometry.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

public class RRConfigTest {
    /**
     * Use the distance metric for position interpolation.
     */
    @Test
    void test0() {
        RRConfig x = new RRConfig(1, 2);
        RRConfig y = new RRConfig(3, 4);
        double d = x.euclideanDistance(y);
        assertEquals(2.828, d, 1e-3);
        // at zero, you get the starting point
        RRConfig x0 = RRConfig.interpolate(x, y, 0);
        TestUtil.verify(x, x0);
        // at full distance, you get the end point.
        RRConfig y0 = RRConfig.interpolate(x, y, d);
        TestUtil.verify(y, y0);
        // halfway is halfway
        RRConfig h = RRConfig.interpolate(x, y, d / 2);
        TestUtil.verify(new RRConfig(2, 3), h);

        // interpolation can also be done by scaling the unit vector
        Vector<N2> unit = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(0.707, 0.707), unit);
        RRConfig x00 = RRConfig.fromVector(x.toVector().plus(unit.times(0)));
        TestUtil.verify(x, x00);
        RRConfig y00 = RRConfig.fromVector(x.toVector().plus(unit.times(d)));
        TestUtil.verify(y, y00);
        RRConfig hh = RRConfig.fromVector(x.toVector().plus(unit.times(d / 2)));
        TestUtil.verify(new RRConfig(2, 3), hh);
    }

    @Test
    void test1() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(1, 0);
        double d = x.euclideanDistance(y);
        assertEquals(1, d, 1e-3);
    }

    @Test
    void test2() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(0, 1);
        double d = x.euclideanDistance(y);
        assertEquals(1, d, 1e-3);
    }

}
