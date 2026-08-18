package org.team100.lib.kinematics.rr;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.Poe;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Test cases from POE.md */
public class RRKinematicsPoETest {

    @Test
    void test00() {
        TestUtil.verify(new Twist2d(0, 0, 1), Poe.S(new Translation2d(0, 0)));
        TestUtil.verify(new Twist2d(0, -1, 1), Poe.S(new Translation2d(1, 0)));
    }

    @Test
    void testp0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // position
        RRPose p = k.forward(new RRConfig(0, 0));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(2, 0, Rotation2d.kZero)), p);
    }

    @Test
    void testp1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // position
        RRPose p = k.forward(new RRConfig(0, Math.PI / 2));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(1, 1, Rotation2d.kCCW_Pi_2)), p);
    }

    @Test
    void testv1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity but not moving
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        // no velocity
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void testv2() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: extended, rotating at shoulder
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        // +y (more) and +theta
        TestUtil.verify(new VelocitySE2(0, 2, 1), v);
    }

    @Test
    void testv3() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: extended, rotating at elbow
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        // +y (less) and +theta
        TestUtil.verify(new VelocitySE2(0, 1, 1), v);
    }

    @Test
    void testv4() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, not moving
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 0));
        // no movement
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void testv5() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, moving at shoulder
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(1, 0));
        // diagonal motion (and +theta)
        TestUtil.verify(new VelocitySE2(-1, 1, 1), v);
    }

    @Test
    void testv6() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, moving at elbow
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 1));
        // motion -x and +theta
        TestUtil.verify(new VelocitySE2(-1, 0, 1), v);
    }

    @Test
    void testv7() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: both joints bent, moving at elbow
        VelocitySE2 v = k.forward(new RRConfig(Math.PI / 2, Math.PI / 2), new RRVelocity(0, 1));
        // motion -y and +theta
        TestUtil.verify(new VelocitySE2(0, -1, 1), v);
    }

    @Test
    void testForwardV0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity examples from the other test
        VelocitySE2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), xdot);
    }

    @Test
    void testForwardV1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(0, 2, 1), xdot);
    }

    @Test
    void testForwardV2() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, 1, 1), xdot);
    }

    @Test
    void testForwardV3() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), xdot);
    }

    @Test
    void testForwardV4() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(-1, 1, 1), xdot);
    }

    @Test
    void testForwardV5() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, 1, 1), xdot);
    }

    @Test
    void testa0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // bent, rotating at shoulder, no joint accel
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // centripetal pulling in
        TestUtil.verify(new AccelerationSE2(-1, -1, 0), xddot);
    }

    @Test
    void testa1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // as above, bent the other way
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // centripetal pulling in
        TestUtil.verify(new AccelerationSE2(-1, 1, 0), xddot);
    }

    @Test
    void testForwardA0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // acceleration examples from the other test
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(0, 0, 0), xddot);
    }

    @Test
    void testForwardA1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // move shoulder: centripetal towards shoulder
        AccelerationSE2 xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(-1, -1, 0), xddot);
    }

    @Test
    void testForwardA2() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // move elbow: centripetal towards elbow
        AccelerationSE2 xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 1),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(-1, 0, 0), xddot);
    }

    @Test
    void test9() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> J = k.J(new RRConfig(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                2, 1, //
                1, 1), J);
    }

    @Test
    void test9v() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> Jv = k.Jv(new RRConfig(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                0, -1, //
                1, 1), Jv);
    }

    @Test
    void test10() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> J = k.J(new RRConfig(0, Math.PI / 2));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -1, -1, //
                1, 0, //
                1, 1), J);
    }

    @Test
    void test11() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is extended, q1 is turning, q2 is not.
        // centripetal acceleration towards origin
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -2, -1, //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test11v() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is extended, q1 is turning, q2 is not.
        // centripetal acceleration towards origin
        Matrix<N3, N2> Jdotv = k.Jdotv(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 1, //
                0, 0, //
                0, 0), Jdotv);
    }

    @Test
    void test12() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is bent, not moving
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test13() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is bent, moving
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -1, 0, //
                -1, -1, //
                0, 0), Jdot);
    }

    @Test
    void testInverseV0() {
        // singular, motionless.
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(0, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV1() {
        // singular, can't move in x
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(1, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV2() {
        // singular, can still move in y
        // note since we specify no rotation, q1 and q2 are opposite.
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, 0),
                new VelocityR2(0, 1));
        TestUtil.verify(new RRVelocity(0.4, 0.2), qdot);
    }

    @Test
    void testInverseV3() {
        // bent, motionless
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(0, 0));
        TestUtil.verify(new RRVelocity(0, 0), qdot);
    }

    @Test
    void testInverseV4() {
        // bent, moving out
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(1, 0));
        TestUtil.verify(new RRVelocity(0, -1), qdot);
    }

    @Test
    void testInverseV5() {
        // bent, moving up
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(0, 1));
        TestUtil.verify(new RRVelocity(1, -1), qdot);
    }

    @Test
    void testInverseV6() {
        // bent, moving diagonally
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRVelocity qdot = k.inverse(
                new RRConfig(0, Math.PI / 2),
                new VelocityR2(-1, 1));
        TestUtil.verify(new RRVelocity(1, 0), qdot);
    }

    @Test
    void testInverseA0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
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
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);

        RRConfig expectedQ = new RRConfig(0, Math.PI / 2);
        RRVelocity expectedQdot = new RRVelocity(0, -1);
        RRAcceleration expectedQddot = new RRAcceleration(1, -1);

        VelocityR2 expectedXdot = new VelocityR2(1, 0);
        AccelerationR2 expectedXddot = new AccelerationR2(0, 0);

        RRVelocity qdot = k.inverse(expectedQ, expectedXdot);
        TestUtil.verify(expectedQdot, qdot);
        RRAcceleration qddot = k.inverse(expectedQ, expectedXdot, expectedXddot);
        TestUtil.verify(expectedQddot, qddot);
        AccelerationSE2 xddot = k.forward(expectedQ, expectedQdot, expectedQddot);
        TestUtil.verify(new AccelerationSE2(0, 0, 0), xddot);
    }

    @Test
    void testInverseA2() {
        // bent, steady +y
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);

        RRConfig expectedQ = new RRConfig(0, Math.PI / 2);
        RRVelocity expectedQdot = new RRVelocity(1, -1);
        RRAcceleration expectedQddot = new RRAcceleration(0, -1);

        VelocityR2 expectedXdot = new VelocityR2(0, 1);
        AccelerationR2 expectedXddot = new AccelerationR2(0, 0);

        RRVelocity qdot = k.inverse(expectedQ, expectedXdot);
        TestUtil.verify(expectedQdot, qdot);
        RRAcceleration qddot = k.inverse(expectedQ, expectedXdot, expectedXddot);
        TestUtil.verify(expectedQddot, qddot);
        AccelerationSE2 xddot = k.forward(expectedQ, expectedQdot, expectedQddot);
        // theta accel
        TestUtil.verify(new AccelerationSE2(0, 0, -1), xddot);
    }
}
