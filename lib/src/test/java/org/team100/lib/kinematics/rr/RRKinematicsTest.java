package org.team100.lib.kinematics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

public class RRKinematicsTest {

    @Test
    void testInfeasible0() {
        // long l1, short l2 => can't reach near origin
        RRKinematics k = new RRKinematics(1, 0.5);
        List<RRConfig> qq = k.inverse(new Translation2d(0.1, 0.1), null);
        assertEquals(0, qq.size());
    }

    @Test
    void testSingular0() {
        // equal links, target at origin => infinite solutions.
        RRKinematics k = new RRKinematics(1, 1);
        assertThrows(IllegalArgumentException.class, () -> k.inverse(new Translation2d(0, 0), null));
    }

    @Test
    void testSingular1() {
        // equal links, target at origin => use default
        RRKinematics k = new RRKinematics(1, 1);
        List<RRConfig> qq = k.inverse(new Translation2d(0, 0), 1.0);
        assertEquals(1, qq.size());
        TestUtil.verify(new RRConfig(1, 3.141), qq.get(0));
    }

    @Test
    void testf1() {
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(0, 0);

        RRPosition actual1 = k.forward(q);
        TestUtil.verify(new Translation2d(1, 0), actual1.p1());
        TestUtil.verify(new Translation2d(2, 0), actual1.p2());
    }

    @Test
    void testi1() {
        RRKinematics k = new RRKinematics(1, 1);

        List<RRConfig> qq = k.inverse(new Translation2d(2, 0), null);
        assertEquals(1, qq.size());
        TestUtil.verify(new RRConfig(0, 0), qq.get(0));
    }

    @Test
    void testf2() {
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(Math.PI / 2, -1 * Math.PI / 2);

        RRPosition actual1 = k.forward(q);
        TestUtil.verify(new Translation2d(0, 1), actual1.p1());
        TestUtil.verify(new Translation2d(1, 1), actual1.p2());
    }

    @Test
    void testi2() {
        RRKinematics k = new RRKinematics(1, 1);

        List<RRConfig> qq = k.inverse(new Translation2d(1, 1), null);
        assertEquals(2, qq.size());

        TestUtil.verify(new RRConfig(Math.PI / 2, -Math.PI / 2), qq.get(0));
        TestUtil.verify(new RRConfig(0, Math.PI / 2), qq.get(1));
    }

    @Test
    void testf3() {
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(Math.PI / 2, -2 * Math.PI / 3);

        RRPosition actual1 = k.forward(q);
        TestUtil.verify(new Translation2d(0, 1), actual1.p1());
        TestUtil.verify(new Translation2d(Math.sqrt(3) / 2, 0.5), actual1.p2());
    }

    @Test
    void testi3() {
        RRKinematics k = new RRKinematics(1, 1);
        List<RRConfig> qq = k.inverse(new Translation2d(Math.sqrt(3) / 2, 0.5), null);
        assertEquals(2, qq.size());

        TestUtil.verify(new RRConfig(Math.PI / 2, -2.094), qq.get(0));
        TestUtil.verify(new RRConfig(-0.523, 2.094), qq.get(1));
    }

    @Test
    void test4() {
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(5 * Math.PI / 6, -2 * Math.PI / 3);

        RRPosition actual1 = k.forward(q);
        TestUtil.verify(new Translation2d(-Math.sqrt(3) / 2, 0.5), actual1.p1());
        TestUtil.verify(new Translation2d(0, 1), actual1.p2());
    }

    @Test
    void testi4() {
        RRKinematics k = new RRKinematics(1, 1);

        List<RRConfig> qq = k.inverse(new Translation2d(0, 1), null);
        assertEquals(2, qq.size());

        TestUtil.verify(new RRConfig(2.618, -2.094), qq.get(0));
        TestUtil.verify(new RRConfig(0.524, 2.094), qq.get(1));
    }

    @Test
    void test5() {
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(Math.PI, -Math.PI / 2);

        RRPosition actual1 = k.forward(q);
        TestUtil.verify(new Translation2d(-1, 0), actual1.p1());
        TestUtil.verify(new Translation2d(-1, 1), actual1.p2());
    }

    @Test
    void testi5() {
        RRKinematics k = new RRKinematics(1, 1);

        List<RRConfig> qq = k.inverse(new Translation2d(-1, 1), null);
        assertEquals(2, qq.size());

        TestUtil.verify(new RRConfig(3.141, -1.571), qq.get(0));
        TestUtil.verify(new RRConfig(1.571, 1.571), qq.get(1));
    }

    @Test
    void testForwardV0() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
    }

    @Test
    void testForwardV1() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(0, 2), xdot);
    }

    @Test
    void testForwardV2() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(0, 1), xdot);
    }

    @Test
    void testForwardV3() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
    }

    @Test
    void testForwardV4() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(-1, 1), xdot);
    }

    @Test
    void testForwardV5() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(0, 1), xdot);
    }

    @Test
    void testInverseV0() {
        // singular, motionless.
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(0, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV1() {
        // singular, can't move in x
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(1, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV2() {
        // singular, can still move in y
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(0, 1));
        TestUtil.verify(new RRVelocity(0.4, 0.2), qdot);
    }

    @Test
    void testInverseV3() {
        // bent, motionless
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(0, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV4() {
        // bent, moving out
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(1, 0));
        TestUtil.verify(new RRVelocity(0, -1), qdot);
    }

    @Test
    void testInverseV5() {
        // bent, moving up
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(0, 1));
        TestUtil.verify(new RRVelocity(1, -1), qdot);
    }

    @Test
    void testInverseV6() {
        // bent, moving diagonally
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(-1, 1));
        TestUtil.verify(new RRVelocity(1, 0), qdot);
    }

    @Test
    void testInverseA0() {
        RRKinematics k = new RRKinematics(1, 1);
        // bent, motionless.
        RRAcceleration qddot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(0, 0),
                new AccelerationR2(0, 0));
        TestUtil.verify(new RRAcceleration(0, 0), qddot);
    }

    @Test
    void testInverseA1() {
        // bent, elbow moving out
        RRKinematics k = new RRKinematics(1, 1);

        RRConfig expectedQ = new RRConfig(0, Math.PI / 2);
        RRVelocity expectedQdot = new RRVelocity(0, -1);
        RRAcceleration expectedQddot = new RRAcceleration(1, -1);

        VelocityR2 expectedXdot = new VelocityR2(1, 0);
        AccelerationR2 expectedXddot = new AccelerationR2(0, 0);

        RRVelocity qdot = k.inverse(expectedQ, expectedXdot);
        TestUtil.verify(expectedQdot, qdot);
        RRAcceleration qddot = k.inverse(expectedQ, expectedXdot, expectedXddot);
        TestUtil.verify(expectedQddot, qddot);
        AccelerationR2 xddot = k.forward(expectedQ, expectedQdot, expectedQddot);
        TestUtil.verify(expectedXddot, xddot);
    }

    @Test
    void testInverseA2() {
        // bent, steady +y
        RRKinematics k = new RRKinematics(1, 1);

        RRConfig expectedQ = new RRConfig(0, Math.PI / 2);
        RRVelocity expectedQdot = new RRVelocity(1, -1);
        RRAcceleration expectedQddot = new RRAcceleration(0, -1);

        VelocityR2 expectedXdot = new VelocityR2(0, 1);
        AccelerationR2 expectedXddot = new AccelerationR2(0, 0);

        RRVelocity qdot = k.inverse(expectedQ, expectedXdot);
        TestUtil.verify(expectedQdot, qdot);
        RRAcceleration qddot = k.inverse(expectedQ, expectedXdot, expectedXddot);
        TestUtil.verify(expectedQddot, qddot);
        AccelerationR2 xddot = k.forward(expectedQ, expectedQdot, expectedQddot);
        TestUtil.verify(expectedXddot, xddot);

    }

    @Test
    void testForwardA0() {
        // singular, motionless
        RRKinematics k = new RRKinematics(1, 1);
        AccelerationR2 xddot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(0, 0), xddot);
    }

    @Test
    void testForwardA1() {
        // move shoulder: centripetal towards shoulder
        RRKinematics k = new RRKinematics(1, 1);
        AccelerationR2 xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(-1, -1), xddot);
    }

    @Test
    void testForwardA2() {
        // move elbow: centripetal towards elbow
        RRKinematics k = new RRKinematics(1, 1);
        AccelerationR2 xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 1),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(-1, 0), xddot);
    }

    @Test
    void testJ0() {
        // singular
        RRKinematics k = new RRKinematics(1, 1);
        RRConfig q = new RRConfig(0, 0);
        Matrix<N2, N2> J = k.J(q);
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, 0, //
                2, 1), J);
    }

    @Test
    void test11() {
        // arm is extended, q1 is turning, q2 is not
        // centripetal acceleration towards origin
        RRKinematics k = new RRKinematics(1, 1);
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -2, -1, //
                0, 0), Jdot);
    }

    @Test
    void test12() {
        // bent, motionless
        RRKinematics k = new RRKinematics(1, 1);
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test13() {
        // bent, moving
        RRKinematics k = new RRKinematics(1, 1);
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -1, 0, //
                -1, -1), Jdot);
    }
}