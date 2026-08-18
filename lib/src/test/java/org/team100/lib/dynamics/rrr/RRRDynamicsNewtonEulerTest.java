package org.team100.lib.dynamics.rrr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.dynamics.rr.RRDynamicsNewtonEuler;
import org.team100.lib.dynamics.rr.RREffort;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;

import edu.wpi.first.math.VecBuilder;

public class RRRDynamicsNewtonEulerTest {
    private static final double DELTA = 1e-3;

    @Test
    void test0() {
        RRRDynamicsNewtonEuler d = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(-9.8, 0, 0),
                1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 1, 1, 1);
        // straight up
        RRREffort t = d.effort(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        // no torques
        assertEquals(0, t.t1(), DELTA);
        assertEquals(0, t.t2(), DELTA);
        assertEquals(0, t.t3(), DELTA);
    }

    @Test
    void test0a() {
        RRRDynamicsNewtonEuler d = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(-9.8, 0, 0),
                1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 1, 1, 1);
        RRRAcceleration t = d.qddot(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRREffort(0, 0, 0));
        assertEquals(0, t.q1ddot(), DELTA);
        assertEquals(0, t.q2ddot(), DELTA);
        assertEquals(0, t.q3ddot(), DELTA);
    }

    @Test
    void test1() {
        RRRDynamicsNewtonEuler d = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(-9.8, 0, 0),
                1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 1, 1, 1);
        // to the side
        RRREffort t = d.effort(
                new RRRConfig(Math.PI / 2, 0, 0),
                new RRRVelocity(0, 0, 0),
                new RRRAcceleration(0, 0, 0));
        // 1 kg is 0.5 m away, so 5Nm, 1 kg 1.5 m away so 15Nm,
        // 1 kg is 2.5 m away so 25 Nm.
        assertEquals(-44.1, t.t1(), DELTA);
        // 1 kg 0.5 m away, 1 kg 1.5 m away
        assertEquals(-19.6, t.t2(), DELTA);
        // 1 kg 0.5 m away
        assertEquals(-4.9, t.t3(), DELTA);
    }

    @Test
    void test3() {
        RRRDynamicsNewtonEuler d = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(-9.8, 0, 0),
                1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 1, 1, 1);
        // bent arm moving at the root
        RRREffort t = d.effort(
                new RRRConfig(0, Math.PI / 2, 0),
                new RRRVelocity(1, 0, 0),
                new RRRAcceleration(0, 0, 0));
        // 1 kg 0.5 m away so 5Nm, 1 kg 1.5m away so 15
        assertEquals(-19.6, t.t1(), DELTA);
        // same as above minus centrifugal force
        assertEquals(-17.6, t.t2(), DELTA);
        // like test 1 but minus centrifugal force
        assertEquals(-4.4, t.t3(), DELTA);
    }

    @Test
    void test5() {
        RRRDynamicsNewtonEuler d = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(-9.8, 0, 0),
                1, 1, 1, 1, 1, 1, 0.5, 0.5, 0.5, 1, 1, 1);
        // like a whip: extended, moving, slowing down at the root
        RRREffort t = d.effort(
                new RRRConfig(0, 0, 0),
                new RRRVelocity(1, 0, 0),
                new RRRAcceleration(-1, 0, 0));
        // elbow tries to keep going, so push back
        assertEquals(-11.75, t.t1(), DELTA);
        // trying to slow down
        assertEquals(-6.5, t.t2(), DELTA);
        assertEquals(-2.25, t.t3(), DELTA);
    }

}
