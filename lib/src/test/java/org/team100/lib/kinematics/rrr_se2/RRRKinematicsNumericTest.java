package org.team100.lib.kinematics.rrr_se2;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

public class RRRKinematicsNumericTest {
    @Test
    void testExtended() {
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        Vector<N3> q = VecBuilder.fill(0, 0, 0);
        Pose3d x = k.forward(q);
        TestUtil.verify(new Pose3d(3, 0, 0, Rotation3d.kZero), x);
    }

    @Test
    void testElbowUp() {
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        // "up" is negative rotation about Y
        Vector<N3> q = VecBuilder.fill(-Math.PI / 2, Math.PI / 2, 0);
        Pose3d x = k.forward(q);
        TestUtil.verify(new Pose3d(2, 0, 1, new Rotation3d(0, 0, 0)), x);
    }

    @Test
    void testMid() {
        // a pose in the middle of the envelope
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        Vector<N3> q = VecBuilder.fill(-3 * Math.PI / 4, Math.PI / 2, 3 * Math.PI / 8);
        Pose3d x = k.forward(q);
        TestUtil.verify(new Pose3d(0.923879, 0, 1.031530, new Rotation3d(0, 0.392699, 0)), x);
    }

    @Test
    void testInverseExtended() {
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        Pose3d x = new Pose3d(3, 0, 0, new Rotation3d());
        Vector<N3> q = k.inverse(x);
        TestUtil.verify(VecBuilder.fill(0, 0, 0), q);
    }

    @Test
    void testInverseElbowUp() {
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        Pose3d x = new Pose3d(2, 0, 1, new Rotation3d());
        Vector<N3> q = k.inverse(x);
        TestUtil.verify(VecBuilder.fill(-Math.PI / 2, Math.PI / 2, 0), q);
    }

    @Test
    void testInverseMid() {
        RRRKinematicsNumeric k = new RRRKinematicsNumeric(1, 1, 1);
        Pose3d x = new Pose3d(0.923879, 0, 1.031530, new Rotation3d(0, 0.392699, 0));
        Vector<N3> q = k.inverse(x);
        TestUtil.verify(VecBuilder.fill(-2.356, 1.571, 1.178), q);
    }

}
