package org.team100.lib.kinematics.pr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.pr.PRAcceleration;
import org.team100.lib.geometry.pr.PRConfig;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

public class PRKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testForward0() {
        PRKinematics k = new PRKinematics(1);
        // at the origin, arm points 1m up
        Translation2d f = k.forward(new PRConfig(0, 0));
        assertEquals(1, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
    }

    @Test
    void testForward1() {
        PRKinematics k = new PRKinematics(1);
        // raise 1m, so arm is 2m up
        Translation2d f = k.forward(new PRConfig(1, 0));
        assertEquals(2, f.getX(), DELTA);
        assertEquals(0, f.getY(), DELTA);
    }

    @Test
    void testForward2() {
        PRKinematics k = new PRKinematics(1); // straight out
        Translation2d f = k.forward(new PRConfig(0, Math.PI / 2));
        assertEquals(0, f.getX(), DELTA);
        assertEquals(1, f.getY(), DELTA);
    }

    @Test
    void testForward3() {
        PRKinematics k = new PRKinematics(1); // 45 degrees
        Translation2d f = k.forward(new PRConfig(0, Math.PI / 4));
        assertEquals(0.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
    }

    @Test
    void testForward4() {
        PRKinematics k = new PRKinematics(1); // 45 degrees
        Translation2d f = k.forward(new PRConfig(1, Math.PI / 4));
        assertEquals(1.707, f.getX(), DELTA);
        assertEquals(0.707, f.getY(), DELTA);
    }

    @Test
    void testInverse0() {
        PRKinematics k = new PRKinematics(1);
        // at the origin, arm points 1m up (or down)
        List<PRConfig> j = k.inverse(new Translation2d(1, 0));
        assertEquals(2, j.size());
        assertEquals(0, j.get(0).q1(), DELTA);
        assertEquals(0, j.get(0).q2(), DELTA);
        assertEquals(2, j.get(1).q1(), DELTA);
        assertEquals(3.141, MathUtil.angleModulus(j.get(1).q2()), DELTA);
    }

    @Test
    void testInverse1() {
        PRKinematics k = new PRKinematics(1);
        // raise 1m, arm points 2m up
        List<PRConfig> j = k.inverse(new Translation2d(2, 0));
        assertEquals(2, j.size());
        assertEquals(1, j.get(0).q1(), DELTA);
        assertEquals(0, j.get(0).q2(), DELTA);
        assertEquals(3, j.get(1).q1(), DELTA);
        assertEquals(3.141, MathUtil.angleModulus(j.get(1).q2()), DELTA);
    }

    @Test
    void testInverse2() {
        PRKinematics k = new PRKinematics(1);
        // straight out
        List<PRConfig> j = k.inverse(new Translation2d(0, 1));
        assertEquals(1, j.size());
        assertEquals(0, j.get(0).q1(), DELTA);
        assertEquals(Math.PI / 2, j.get(0).q2(), DELTA);
    }

    @Test
    void testInverse3() {
        PRKinematics k = new PRKinematics(1);
        // 45 degrees
        List<PRConfig> j = k.inverse(new Translation2d(0.707107, 0.707107));
        assertEquals(2, j.size());
        assertEquals(0, j.get(0).q1(), DELTA);
        assertEquals(Math.PI / 4, j.get(0).q2(), DELTA);
        assertEquals(1.414, j.get(1).q1(), DELTA);
        assertEquals(3 * Math.PI / 4, j.get(1).q2(), DELTA);
    }

    @Test
    void testInverse4() {
        PRKinematics k = new PRKinematics(1);
        // 45 degrees?
        List<PRConfig> j = k.inverse(new Translation2d(1.707107, 0.707107));
        assertEquals(2, j.size());
        assertEquals(1, j.get(0).q1(), DELTA);
        assertEquals(Math.PI / 4, j.get(0).q2(), DELTA);
        assertEquals(2.414, j.get(1).q1(), DELTA);
        assertEquals(3 * Math.PI / 4, j.get(1).q2(), DELTA);
    }

    @Test
    void testInverse5() {
        PRKinematics k = new PRKinematics(1);
        // these are unreachable
        assertEquals(0, k.inverse(new Translation2d(0, 2)).size());
        assertEquals(0, k.inverse(new Translation2d(0, -2)).size());
    }

    @Test
    void testForwardV0() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
    }

    @Test
    void testForwardV1() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(0, 0), new PRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(1, 0), xdot);
    }

    @Test
    void testForwardV2() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(0, 0), new PRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(0, 1), xdot);
    }

    @Test
    void testForwardV3() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
    }

    @Test
    void testForwardV4() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(1, 0), xdot);
    }

    @Test
    void testForwardV5() {
        PRKinematics k = new PRKinematics(1);
        VelocityR2 xdot = k.forward(new PRConfig(1, Math.PI / 2), new PRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(-1, 0), xdot);
    }

    @Test
    void testInverseV0() {
        // straight up
        PRKinematics k = new PRKinematics(1);
        PRVelocity qdot = k.inverse(
                new PRConfig(1, 0),
                new VelocityR2(0, 0));
        TestUtil.verify(new PRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV1() {
        PRKinematics k = new PRKinematics(1);
        PRVelocity qdot = k.inverse(
                new PRConfig(1, 0),
                new VelocityR2(1, 0));
        TestUtil.verify(new PRVelocity(1, 0), qdot);
    }

    @Test
    void testInverseV2() {
        PRKinematics k = new PRKinematics(1);
        PRVelocity qdot = k.inverse(
                new PRConfig(1, 0),
                new VelocityR2(0, 1));
        TestUtil.verify(new PRVelocity(0, 1), qdot);
    }

    @Test
    void testInverseV3() {
        PRKinematics k = new PRKinematics(1);
        PRVelocity qdot = k.inverse(
                new PRConfig(1, Math.PI / 2),
                new VelocityR2(0, 0));
        TestUtil.verify(new PRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV4() {
        PRKinematics k = new PRKinematics(1);
        // singularity, but can still move in x.
        PRVelocity qdot = k.inverse(
                new PRConfig(1, Math.PI / 2),
                new VelocityR2(1, 0));
        TestUtil.verify(new PRVelocity(0.5, -0.5), qdot);
    }

    @Test
    void testInverseV5() {
        PRKinematics k = new PRKinematics(1);
        // singularity, can't move in y
        PRVelocity qdot = k.inverse(
                new PRConfig(1, Math.PI / 2),
                new VelocityR2(0, 1));
        TestUtil.verify(new PRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV6() {
        PRKinematics k = new PRKinematics(1);
        // singularity, can partially satisfy the velocity requirement.
        PRVelocity qdot = k.inverse(
                new PRConfig(1, Math.PI / 2),
                new VelocityR2(-1, 1));
        TestUtil.verify(new PRVelocity(-0.5, 0.5), qdot);
    }

    @Test
    void testForwardA0() {
        PRKinematics k = new PRKinematics(1);
        AccelerationR2 xddot = k.forward(
                new PRConfig(0, 0),
                new PRVelocity(0, 0),
                new PRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(0, 0), xddot);
    }

    @Test
    void testForwardA1() {
        PRKinematics k = new PRKinematics(1); // centripetal
        AccelerationR2 xddot = k.forward(
                new PRConfig(0, 0),
                new PRVelocity(0, 1),
                new PRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(-1, 0), xddot);
    }

    @Test
    void testInverseA0() {
        PRKinematics k = new PRKinematics(1);
        PRAcceleration qddot = k.inverse(
                new PRConfig(1, 0),
                new VelocityR2(0, 0),
                new AccelerationR2(0, 0));
        TestUtil.verify(new PRAcceleration(0, 0), qddot);
    }

    @Test
    void testInverseA1() {
        PRKinematics k = new PRKinematics(1);
        // P extends to support +y
        PRAcceleration qddot = k.inverse(
                new PRConfig(1, 0),
                new VelocityR2(0, 1),
                new AccelerationR2(0, 0));
        TestUtil.verify(new PRAcceleration(1, 0), qddot);
    }
}
