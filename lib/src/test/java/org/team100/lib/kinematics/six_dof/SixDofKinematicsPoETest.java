package org.team100.lib.kinematics.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se3.AccelerationSE3;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.kinematics.Poe;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;

public class SixDofKinematicsPoETest {
    /**
     * Example from
     * https://github.com/madibabaiasl/modern-robotics-course/wiki/Lesson-7:-Forward-Kinematics-of-Robot-Arms-Using-Screw-Theory
     */
    @Test
    void testS1() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // at the origin
        Translation3d a = new Translation3d(0, 0, 0);
        Twist3d S = Poe.S(So, a);
        TestUtil.verify(new Twist3d(0, 0, 0, 0, 0, 1), S);
    }

    @Test
    void testS2() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // offset by (say) x=1
        Translation3d a = new Translation3d(1, 0, 0);
        Twist3d S = Poe.S(So, a);
        TestUtil.verify(new Twist3d(0, -1, 0, 0, 0, 1), S);
    }

    @Test
    void testS3() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // offset by (say) x=2
        Translation3d a = new Translation3d(2, 0, 0);
        Twist3d S = Poe.S(So, a);
        TestUtil.verify(new Twist3d(0, -2, 0, 0, 0, 1), S);
    }

    @Test
    void testForward1() {
        SixDofKinematicsPoE k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofPose p = k.forward(new SixDofConfig(0, 0, 0, 0, 0, 0));
        TestUtil.verify(new Pose3d(0, 0, 0, Rotation3d.kZero), p.p1());
        TestUtil.verify(new Pose3d(0, 0, 0.25, Rotation3d.kZero), p.p2());
        TestUtil.verify(new Pose3d(0.75, 0, 0.25, Rotation3d.kZero), p.p3());
        TestUtil.verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p4());
        TestUtil.verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p5());
        TestUtil.verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p6());
        // note the zero rotation, different than the other implementation.
        TestUtil.verify(new Pose3d(1.65, 0, 0.25, Rotation3d.kZero), p.p7());
    }

    @Test
    void testForward1a() {
        SixDofKinematicsPoE k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofPose p = k.forward(new SixDofConfig(Math.PI / 2, 0, 0, 0, 0, 0));
        TestUtil.verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        TestUtil.verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p2());
        TestUtil.verify(new Pose3d(0, 0.75, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p3());
        TestUtil.verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p4());
        TestUtil.verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p5());
        TestUtil.verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p6());
        // note the zero rotation, different than the other implementation.
        TestUtil.verify(new Pose3d(0, 1.65, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p7());
    }

    @Test
    void testForward2() {
        // point up
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(
                0, // yaw +x
                Math.PI / 2, // shoulder up
                -Math.PI / 2, // elbow out
                0, // use pitch axis for pitch
                Math.PI / 2, // pitch up
                0);
        SixDofPose p = k.forward(q);
        // tool is pointing up
        TestUtil.verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        TestUtil.verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, 0, 0)), p.p2());
        TestUtil.verify(new Pose3d(0, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p3());
        TestUtil.verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, 0, 0)), p.p4());
        TestUtil.verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, 0, 0)), p.p5());
        TestUtil.verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p6());
        TestUtil.verify(new Pose3d(0.75, 0, 1.15, new Rotation3d(0, -Math.PI / 2, 0)), p.p7());
    }

    @Test
    void testInverse1() {
        // This is the wrist singularity and the elbow singularity
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(1.65, 0, 0.25,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, 1.0);
        assertEquals(2, q.size());
        // the q6 are different because of the default q4
        TestUtil.verify(new SixDofConfig(0, 0, 0, 1, 0, -1), q.get(0));
        TestUtil.verify(new SixDofConfig(3.141, 3.141, 0, 1, 0, 2.141), q.get(1));
    }

    @Test
    void testInverse2() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global -z
        Pose3d p = new Pose3d(0.5, 0, 0.5,
                new Rotation3d(0, Math.PI / 2, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());
        TestUtil.verify(new SixDofConfig(0, 1.804, -2.259, 0, -1.116, 0), q.get(0));
        TestUtil.verify(new SixDofConfig(0, 1.804, -2.259, 3.141, 1.116, 3.141), q.get(1));
        TestUtil.verify(new SixDofConfig(0, -0.455, 2.259, 3.141, -2.908, 3.141), q.get(2));
        TestUtil.verify(new SixDofConfig(0, -0.455, 2.259, 0, 2.908, 0), q.get(3));

        TestUtil.verify(new SixDofConfig(3.141, -2.687, -2.259, 0, -2.908, 3.141), q.get(4));
        TestUtil.verify(new SixDofConfig(3.141, -2.687, -2.259, 3.141, 2.908, 0), q.get(5));
        TestUtil.verify(new SixDofConfig(3.141, 1.337, 2.259, -3.141, -1.116, 0), q.get(6));
        TestUtil.verify(new SixDofConfig(3.141, 1.337, 2.259, 0, 1.116, 3.141), q.get(7));

    }

    @Test
    void testInverse3() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x, tool x at global -y
        // note position offset +y, wrist should be at (0.5,0.5,0.5)
        Pose3d p = new Pose3d(0.65, 0.5, 0.5,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());

        TestUtil.verify(new SixDofConfig(0.785, 1.387, -2.094, -2.147, -1.003, 2.451), q.get(0));
        TestUtil.verify(new SixDofConfig(0.785, 1.387, -2.094, 0.994, 1.003, -0.691), q.get(1));
        // elbow-down cases
        TestUtil.verify(new SixDofConfig(0.785, -0.707, 2.094, -0.794, -1.441, 0.131), q.get(2));
        TestUtil.verify(new SixDofConfig(0.785, -0.707, 2.094, 2.348, 1.441, -3.011), q.get(3));
    }

    @Test
    void testInverse4a() {
        // This is the base singularity with no default
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> k.inverse(p, null, null));
    }

    @Test
    void testInverse4b() {
        // This is the base singularity with a default, a good example of various
        // solutions using the base default.
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, 1.0, null);
        assertEquals(4, q.size());

        // elbow-up
        TestUtil.verify(new SixDofConfig(1.0, 2.618, -2.094, -1.260, -1.084, 0.969), q.get(0));
        TestUtil.verify(new SixDofConfig(1.0, 2.618, -2.094, 1.881, 1.084, -2.172), q.get(1));
        // elbow-down
        TestUtil.verify(new SixDofConfig(1.0, 0.523, 2.094, -1.260, -2.058, -0.969), q.get(2));
        TestUtil.verify(new SixDofConfig(1.0, 0.523, 2.094, 1.881, 2.058, 2.172), q.get(3));

    }

    @Test
    void testInverse5() {
        // This is reaching "back" behind the base singularity.
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0, 0, 1,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());

        // flip, elbow up
        TestUtil.verify(new SixDofConfig(3.141, 2.409, -2.071, -3.141, -2.804, 0), q.get(0));
        TestUtil.verify(new SixDofConfig(3.141, 2.409, -2.071, 0, 2.804, 3.141), q.get(1));

        // flip, elbow down
        TestUtil.verify(new SixDofConfig(3.141, 0.338, 2.071, -3.141, -0.732, 0), q.get(2));
        TestUtil.verify(new SixDofConfig(3.141, 0.338, 2.071, 0, 0.732, 3.141), q.get(3));

        // noflip, elbow
        TestUtil.verify(new SixDofConfig(0, 2.804, -2.071, 0, -0.732, 0), q.get(4));
        TestUtil.verify(new SixDofConfig(0, 2.804, -2.071, 3.141, 0.732, 3.141), q.get(5));

        TestUtil.verify(new SixDofConfig(0, 0.732, 2.071, 0, -2.804, 0), q.get(6));
        TestUtil.verify(new SixDofConfig(0, 0.732, 2.071, 3.141, 2.804, 3.141), q.get(7));
    }

    @Test
    void testJ0() {
        // jacobian for a case I can figure out
        SixDofKinematicsPoE k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        // TCP is here:
        SixDofPose p = k.forward(q);
        TestUtil.verify(new Pose3d(1.65, 0, 0.25, Rotation3d.kZero), p.p7());
        Matrix<N6, N6> J = k.J(q);
        TestUtil.verify(MatBuilder.fill(Nat.N6(), Nat.N6(),
                0.00, 0.00, 0.00, 0.0, 0.00, 0.0, // zero because singular
                1.65, 0.00, 0.00, 0.0, 0.00, 0.0, // y hears base
                0.00, 1.65, 0.90, 0.0, 0.15, 0.0, // z hears shoulder, elbow, pitch
                0.00, 0.00, 0.00, 1.0, 0.00, 1.0, // roll from roll
                0.00, -1.0, -1.0, 0.0, -1.0, 0.0, // all rot parallel (inverted)
                1.00, 0.00, 0.00, 0.0, 0.00, 0.0), // zero because singular
                J);
    }

    @Test
    void testForwardV0() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // extended
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        // actuate just the shoulder
        SixDofVelocity qdot = new SixDofVelocity(0, 1, 0, 0, 0, 0);
        VelocitySE3 xdot = k.forward(q, qdot);
        // velocity +z, also pitch down so wrist orientation remains the same
        TestUtil.verify(new VelocitySE3(0, 0, 1.65, 0, -1, 0), xdot);
    }

    @Test
    void testForwardV0a() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // extended
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        // actuate just the shoulder
        SixDofVelocity qdot = new SixDofVelocity(1, 0, 0, 0, 0, 0);
        VelocitySE3 xdot = k.forward(q, qdot);
        // velocity +z, also pitch down so wrist orientation remains the same
        TestUtil.verify(new VelocitySE3(0, 1.65, 0, 0, 0, 1), xdot);
    }

    @Test
    void testForwardV1() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // shoulder up, elbow out
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, 0, 0);
        // actuate just the elbow
        SixDofVelocity qdot = new SixDofVelocity(0, 0, 1, 0, 0, 0);
        VelocitySE3 xdot = k.forward(q, qdot);
        // velocity +z, also pitch down so wrist orientation remains the same
        TestUtil.verify(new VelocitySE3(0, 0, 0.9, 0, -1, 0), xdot);
    }

    @Test
    void testForwardV2() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // extended, with wrist up
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, Math.PI / 2, 0);
        // actuate just the shoulder
        SixDofVelocity qdot = new SixDofVelocity(0, 1, 0, 0, 0, 0);
        VelocitySE3 xdot = k.forward(q, qdot);
        // velocity -x (because offset up) and +z, also pitch down so wrist orientation
        // remains the same
        TestUtil.verify(new VelocitySE3(-0.15, 0, 1.5, 0, -1, 0), xdot);
    }

    @Test
    void testInverseV0() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // shoulder up, elbow out, wrist down
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, Math.PI / 2, 0);
        // move up
        VelocitySE3 xdot = new VelocitySE3(0, 0, 1, 0, 0, 0);
        SixDofVelocity qdot = k.inverse(q, xdot);
        // elbow up, wrist down
        TestUtil.verify(new SixDofVelocity(0, 0, 1.333, 0, -1.333, 0), qdot);
    }

    @Test
    void testForwardV3() {
        // the case above
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, Math.PI / 2, 0);
        SixDofVelocity qdot = new SixDofVelocity(0, 0, 1.333, 0, -1.333, 0);
        VelocitySE3 xdot = k.forward(q, qdot);
        TestUtil.verify(new VelocitySE3(0, 0, 1, 0, 0, 0), xdot);
    }

    @Test
    void testForwardA0() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // shoulder up, elbow out, wrist down
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, Math.PI / 2, 0);
        // moving up
        SixDofVelocity qdot = new SixDofVelocity(0, 0, 1, 0, 0, 0);
        // without accelerating
        SixDofAcceleration qddot = new SixDofAcceleration(0, 0, 0, 0, 0, 0);
        AccelerationSE3 xdot = k.forward(q, qdot, qddot);
        TestUtil.verify(new AccelerationSE3(-0.75, 0, -0.15, 0, 0, 0), xdot);
    }

    @Test
    void testInverseA0() {
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // shoulder up, elbow out, wrist down
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, Math.PI / 2, 0);
        // move up
        VelocitySE3 xdot = new VelocitySE3(0, 0, 1, 0, 0, 0);
        SixDofVelocity qdot = k.inverse(q, xdot);
        TestUtil.verify(new SixDofVelocity(0, 0, 1.333, 0, -1.333, 0), qdot);
        AccelerationSE3 xddot = new AccelerationSE3(0, 0, 0, 0, 0, 0);
        SixDofAcceleration qddot = k.inverse(q, xdot, xddot);
        // this is wrong
        TestUtil.verify(new SixDofAcceleration(0, 0, 0, 0, 0, 0), qddot);
    }

    @Test
    void testForwardA1() {
        // the case above
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        // shoulder up, elbow out, wrist down
        SixDofConfig q = new SixDofConfig(0, Math.PI / 2, -Math.PI / 2, 0, Math.PI / 2, 0);
        // moving up
        SixDofVelocity qdot = new SixDofVelocity(0, 0, 1.333, 0, -1.333, 0);
        // without accelerating
        SixDofAcceleration qddot = new SixDofAcceleration(0, 0, 0, 0, 0, 0);
        AccelerationSE3 xdot = k.forward(q, qdot, qddot);
        TestUtil.verify(new AccelerationSE3(0, 0, 0, 0, 0, 0), xdot);
    }

}
