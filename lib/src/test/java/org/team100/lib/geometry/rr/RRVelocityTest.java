package org.team100.lib.geometry.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;



public class RRVelocityTest {
    /**
     * Use the distance metric for velocity scaling.
     */
    @Test
    void test0() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(1, 0);
        double d = x.distance(y);
        assertEquals(1.732, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(0.577, 0), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        // If the pathwise velocity is just the distance,
        // then the joint velocity should be 1
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(1, 0), v);
    }

    @Test
    void test1() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(2, 0);
        double d = x.distance(y);
        assertEquals(3.464, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(0.577, 0), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(2, 0), v);
    }

    @Test
    void test2() {
        RRConfig x = new RRConfig(0, 0);
        // the position here is inverse to the weights, so the weighted distance is just
        // the euclidean distance from (0,0) to (1,1)
        RRConfig y = new RRConfig(Math.sqrt(1.0 / 3), Math.sqrt(1.0 / 2));
        TestUtil.verify(new RRConfig(0.577, 0.707), y);
        double d = x.distance(y);
        assertEquals(1.414, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(0.408, 0.5), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(0.577, 0.707), v);
    }

    @Test
    void test3() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(1, 1);
        double d = x.distance(y);
        assertEquals(2.236, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        // the unit vector
        TestUtil.verify(VecBuilder.fill(0.447, 0.447), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(1, 1), v);
    }

}
