package org.team100.lib.kinematics.rrr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

public class RRRKinematicsTest {
    @Test
    void testExtended() {
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        RRRConfig q = new RRRConfig(0, 0, 0);
        Pose3d x = k.forward(q);
        assertEquals(3, x.getX(), 1e-3);
        assertEquals(0, x.getY(), 1e-3);
        assertEquals(0, x.getZ(), 1e-3);
        assertEquals(0, x.getRotation().getX(), 1e-3);
        assertEquals(0, x.getRotation().getY(), 1e-3);
        assertEquals(0, x.getRotation().getZ(), 1e-3);
    }

    @Test
    void testElbowUp() {
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        // "up" is negative rotation about Y
        RRRConfig q = new RRRConfig(-Math.PI / 2, Math.PI / 2, 0);
        Pose3d x = k.forward(q);
        assertEquals(2, x.getX(), 1e-3);
        assertEquals(0, x.getY(), 1e-3);
        assertEquals(1, x.getZ(), 1e-3);
        assertEquals(0, x.getRotation().getX(), 1e-3);
        assertEquals(0, x.getRotation().getY(), 1e-3);
        assertEquals(0, x.getRotation().getZ(), 1e-3);
    }

    @Test
    void testMid() {
        // a pose in the middle of the envelope
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        RRRConfig q = new RRRConfig(-3 * Math.PI / 4, Math.PI / 2, 3 * Math.PI / 8);
        Pose3d x = k.forward(q);
        assertEquals(0.923879, x.getX(), 1e-6);
        assertEquals(0, x.getY(), 1e-6);
        assertEquals(1.031530, x.getZ(), 1e-6);
        assertEquals(0, x.getRotation().getX(), 1e-6);
        assertEquals(0.392699, x.getRotation().getY(), 1e-6);
        assertEquals(0, x.getRotation().getZ(), 1e-6);
    }

    @Test
    void testInverseExtended() {
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        Pose3d x = new Pose3d(3, 0, 0, new Rotation3d());
        RRRConfig q = k.inverse(x);
        assertEquals(0, q.q1(), 1e-3);
        assertEquals(0, q.q2(), 1e-3);
        assertEquals(0, q.q3(), 1e-3);
    }

    @Test
    void testInverseElbowUp() {
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        Pose3d x = new Pose3d(2, 0, 1, new Rotation3d());
        RRRConfig q = k.inverse(x);
        assertEquals(-Math.PI / 2, q.q1(), 1e-3);
        assertEquals(Math.PI / 2, q.q2(), 1e-3);
        assertEquals(0, q.q3(), 1e-3);
    }

    @Test
    void testInverseMid() {
        RRRKinematics k = new RRRKinematics(1, 1, 1);
        Pose3d x = new Pose3d(0.923879, 0, 1.031530, new Rotation3d(0, 0.392699, 0));
        RRRConfig q = k.inverse(x);
        assertEquals(-2.356, q.q1(), 1e-3);
        assertEquals(1.571, q.q2(), 1e-3);
        assertEquals(1.178, q.q3(), 1e-3);
    }

}
