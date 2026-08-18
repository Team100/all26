package org.team100.lib.dynamics.six_dof;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.VecBuilder;

public class SixDofDynamicsNewtonEulerTest {
    @Test
    void test0() {
        SixDofDynamicsNewtonEuler d = new SixDofDynamicsNewtonEuler(
                0.25, 0.75, 0.75, 0.15, 1, 1, 1, 1);
        // point straight up
        SixDofEffort t = d.effort(
                new SixDofConfig(0, Math.PI / 2, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofAcceleration(0, 0, 0, 0, 0, 0));
        TestUtil.verify(VecBuilder.fill(0, 0, 0, 0, 0, 0), t.toVector());
    }

        @Test
    void test0a() {
        SixDofDynamicsNewtonEuler d = new SixDofDynamicsNewtonEuler(
                0.25, 0.75, 0.75, 0.15, 1, 1, 1, 1);
        // point straight up
        SixDofEffort t = d.effort(
                new SixDofConfig(0, 0, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofAcceleration(0, 0, 0, 0, 0, 0));
        TestUtil.verify(VecBuilder.fill(0, 33.075, 13.23, 0, 0.735, 0), t.toVector());
    }

    @Test
    void test1() {
        SixDofDynamicsNewtonEuler d = new SixDofDynamicsNewtonEuler(
                0.25, 0.75, 0.75, 0.15, 1, 1, 1, 1);
        // point straight up
        SixDofAcceleration qddot = d.qddot(
                new SixDofConfig(0, Math.PI / 2, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofEffort(0, 0, 0, 0, 0, 0));
        TestUtil.verify(VecBuilder.fill(0, 0, 0, 0, 0, 0), qddot.toVector());
    }

    @Test
    void test1a() {
        SixDofDynamicsNewtonEuler d = new SixDofDynamicsNewtonEuler(
                0.25, 0.75, 0.75, 0.15, 1, 1, 1, 1);
        // point straight up
        SixDofAcceleration qddot = d.qddot(
                new SixDofConfig(0, 0, 0, 0, 0, 0),
                new SixDofVelocity(0, 0, 0, 0, 0, 0),
                new SixDofEffort(0, 0, 0, 0, 0, 0));
        TestUtil.verify(VecBuilder.fill(0, -16.319, 19.894, 0, -3.744, 0), qddot.toVector());
    }

}
