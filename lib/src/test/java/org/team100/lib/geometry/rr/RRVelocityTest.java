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
        double d = x.euclideanDistance(y);
        assertEquals(1, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(1, 0), u);
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
        double d = x.euclideanDistance(y);
        assertEquals(2, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        TestUtil.verify(VecBuilder.fill(1, 0), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(2, 0), v);
    }

    @Test
    void test3() {
        RRConfig x = new RRConfig(0, 0);
        RRConfig y = new RRConfig(1, 1);
        double d = x.euclideanDistance(y);
        assertEquals(1.414, d, 1e-3);
        Vector<N2> u = RRConfig.unit(x, y);
        // the unit vector
        TestUtil.verify(VecBuilder.fill(0.707, 0.707), u);
        assertEquals(1, RRVelocity.fromVector(u).norm(), 1e-3);
        RRVelocity v = RRVelocity.fromVector(u.times(d));
        TestUtil.verify(new RRVelocity(1, 1), v);
    }

}
