package org.team100.lib.kinematics.rrr_se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRPose;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

public class RRRKinematicsPoETest {
    @Test
    void test0a() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(0, 0, 0);
        RRRPose x = k.forward(q);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), x.p1());
        TestUtil.verify(new Pose2d(1, 0, Rotation2d.kZero), x.p2());
        TestUtil.verify(new Pose2d(2, 0, Rotation2d.kZero), x.p3());
        TestUtil.verify(new Pose2d(3, 0, Rotation2d.kZero), x.p4());
    }

    @Test
    void test0b() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(Math.PI / 2, -Math.PI / 2, Math.PI / 2);
        RRRPose x = k.forward(q);
        TestUtil.verify(new Pose2d(0, 0, Rotation2d.kZero), x.p1());
        TestUtil.verify(new Pose2d(0, 1, Rotation2d.kCCW_Pi_2), x.p2());
        TestUtil.verify(new Pose2d(1, 1, Rotation2d.kZero), x.p3());
        TestUtil.verify(new Pose2d(1, 2, Rotation2d.kCCW_Pi_2), x.p4());
    }

    @Test
    void test1a() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(0, 0, 0);
        RRRVelocity qdot = new RRRVelocity(1, 0, 0);
        VelocitySE2 xdot = k.forward(q, qdot);
        TestUtil.verify(new VelocitySE2(0, 3, 1), xdot);
    }

    @Test
    void test1b() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(Math.PI / 2, -Math.PI / 2, Math.PI / 2);
        RRRVelocity qdot = new RRRVelocity(1, 0, 0);
        VelocitySE2 xdot = k.forward(q, qdot);
        TestUtil.verify(new VelocitySE2(-2, 1, 1), xdot);
    }

    @Test
    void test2() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(0, 0, 0);
        RRRVelocity qdot = new RRRVelocity(1, 0, 0);
        RRRAcceleration qddot = new RRRAcceleration(0, 0, 0);
        AccelerationSE2 xddot = k.forward(q, qdot, qddot);
        // centripetal
        TestUtil.verify(new AccelerationSE2(-3, 0, 0), xddot);
    }

    @Test
    void test3a() {
        // equilateral triangle, two ways.
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        Pose2d x = new Pose2d();
        List<RRRConfig> q = k.inverse(x, null);
        assertEquals(2, q.size());
        TestUtil.verify(new RRRConfig(-2.094, -2.094, -2.094), q.get(0));
        TestUtil.verify(new RRRConfig(2.094, 2.094, 2.094), q.get(1));
    }

    @Test
    void test3b() {
        // indeterminate q1.
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        Pose2d x = new Pose2d(1, 0, Rotation2d.kZero);
        List<RRRConfig> q = k.inverse(x, 1.0);
        assertEquals(1, q.size());
        TestUtil.verify(new RRRConfig(1.0, 3.141, 2.141), q.get(0));
    }

    @Test
    void test3c() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        Pose2d x = new Pose2d(1, 0, Rotation2d.kCW_Pi_2);
        List<RRRConfig> q = k.inverse(x, null);
        assertEquals(2, q.size());
        TestUtil.verify(new RRRConfig(Math.PI / 2, -Math.PI / 2, -Math.PI / 2), q.get(0));
        TestUtil.verify(new RRRConfig(0, Math.PI / 2, -Math.PI), q.get(1));
    }

    @Test
    void test4a() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // extended (singular)
        RRRConfig q = new RRRConfig(0, 0, 0);
        // move in y only
        VelocitySE2 xdot = new VelocitySE2(0, 1, 0);
        RRRVelocity qdot = k.inverse(q, xdot);
        // move shoulder only and compensate for rotation with wrist
        TestUtil.verify(new RRRVelocity(0.5, 0, -0.5), qdot);
    }

    @Test
    void test4b() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // not singular
        RRRConfig q = new RRRConfig(Math.PI / 2, -Math.PI / 2, Math.PI / 2);
        // move -x
        VelocitySE2 xdot = new VelocitySE2(-1, 0, 0);
        RRRVelocity qdot = k.inverse(q, xdot);
        // move q1, compensate with q2
        TestUtil.verify(new RRRVelocity(1, -1, 0), qdot);
    }

    @Test
    void test4c() {
        // TODO: ???
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // extended (singular)
        RRRConfig q = new RRRConfig(0, 0, 0);
        //
        VelocitySE2 xdot = new VelocitySE2(0, 1, 0.333);
        RRRVelocity qdot = k.inverse(q, xdot);
        //
        TestUtil.verify(new RRRVelocity(0.278, 0.111, -0.056), qdot);
    }

    @Test
    void test5a() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(Math.PI / 2, -Math.PI / 2, Math.PI / 2);
        VelocitySE2 xdot = new VelocitySE2(0, 0, 0);
        AccelerationSE2 xddot = new AccelerationSE2(-1, 0, 0);
        RRRAcceleration qdot = k.inverse(q, xdot, xddot);
        // since it's not moving, accel is like velo above
        TestUtil.verify(new RRRAcceleration(1, -1, 0), qdot);
    }

    @Test
    void test5b() {
        // TODO: ???
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        RRRConfig q = new RRRConfig(Math.PI / 2, -Math.PI / 2, Math.PI / 2);
        //
        VelocitySE2 xdot = new VelocitySE2(-1, 0, 0);
        AccelerationSE2 xddot = new AccelerationSE2(0, 0, 0);
        RRRAcceleration qdot = k.inverse(q, xdot, xddot);
        //
        TestUtil.verify(new RRRAcceleration(0, 1, -1), qdot);
    }

    @Test
    void test5c() {
        // TODO: ???
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // extended
        RRRConfig q = new RRRConfig(0, 0, 0);
        //
        VelocitySE2 xdot = new VelocitySE2(0, 1, 1);
        AccelerationSE2 xddot = new AccelerationSE2(0, 0, 0);
        RRRAcceleration qdot = k.inverse(q, xdot, xddot);
        //
        TestUtil.verify(new RRRAcceleration(0, 0, 0), qdot);
    }

    @Test
    void testJ0() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // fully extended
        RRRConfig q = new RRRConfig(0, 0, 0);
        Matrix<N3, N3> J = k.J(q);
        // each joint affects y but not x
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, 0, 0, //
                3, 2, 1, //
                1, 1, 1), J);
    }

    @Test
    void testJinv0() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(1, 1, 1);
        // fully extended
        RRRConfig q = new RRRConfig(0, 0, 0);
        Matrix<N3, N3> Jinv = k.Jinv(q);
        // kind of arbitrary choices here
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, 0.5, -0.667, //
                0, 0, 0.333, //
                0, -0.5, 1.333), Jinv);
    }
}
