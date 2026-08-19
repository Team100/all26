package org.team100.lib.kinematics.pprrrr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.pprrrr.PPRRRRConfig;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

public class PPRRRRKinematicsTest {
    @Test
    void test0() {
        PPRRRRKinematics k = new PPRRRRKinematics(1, 1, 1);
        PPRRRRConfig q = new PPRRRRConfig(0, 0, 0, 0, 0, 0);
        Pose3d x = k.forward(q);
        assertEquals(3, x.getX(), 1e-3);
        assertEquals(0, x.getY(), 1e-3);
        assertEquals(0, x.getZ(), 1e-3);
        assertEquals(0, x.getRotation().getX(), 1e-3);
        assertEquals(0, x.getRotation().getY(), 1e-3);
        assertEquals(0, x.getRotation().getZ(), 1e-3);
    }

    @Test
    void testMid() {
        // a pose in the middle of the envelope
        PPRRRRKinematics k = new PPRRRRKinematics(1, 1, 1);
        PPRRRRConfig q = new PPRRRRConfig(
                2, 3, 0, -3 * Math.PI / 4, Math.PI / 2, 3 * Math.PI / 8);
        Pose3d x = k.forward(q);
        // note displacement
        assertEquals(2.923879, x.getX(), 1e-6);
        assertEquals(3, x.getY(), 1e-6);
        assertEquals(1.031530, x.getZ(), 1e-6);
        assertEquals(0, x.getRotation().getX(), 1e-6);
        assertEquals(0.392699, x.getRotation().getY(), 1e-6);
        assertEquals(0, x.getRotation().getZ(), 1e-6);
    }

    // TODO: figure out why this test fails
    //@Test
    void testInverse() {
        // in the middle of the field, looking down
        PPRRRRKinematics k = new PPRRRRKinematics(1, 1, 1);
        Pose3d x = new Pose3d(8, 4, 0, new Rotation3d(0, Math.PI / 2, 0));
        PPRRRRConfig q = k.inverse(x);
        assertEquals(8, q.q1(), 1e-3);
        assertEquals(3, q.q2(), 1e-3);
        assertEquals(Math.PI / 2, q.q3(), 1e-3);
        assertEquals(-Math.PI / 2, q.q4(), 1e-3);
        assertEquals(Math.PI / 2, q.q5(), 1e-3);
        assertEquals(0, q.q6(), 1e-3);

    }
}
