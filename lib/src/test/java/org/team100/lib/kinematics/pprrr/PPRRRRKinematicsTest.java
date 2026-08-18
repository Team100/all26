package org.team100.lib.kinematics.pprrr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.pprrr.PPRRRConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PPRRRRKinematicsTest {
    @Test
    void testForward() {
        PPRRRKinematics k = new PPRRRKinematics(1, 1);
        PPRRRConfig q = new PPRRRConfig(0, 0, 0, 0, 0);
        Pose3d x = k.forward(q);
        assertEquals(2, x.getX(), 1e-3);
        assertEquals(0, x.getY(), 1e-3);
        assertEquals(0, x.getZ(), 1e-3);
        assertEquals(0, x.getRotation().getX(), 1e-3);
        assertEquals(0, x.getRotation().getY(), 1e-3);
        assertEquals(0, x.getRotation().getZ(), 1e-3);
    }

    @Test
    void testForwardMid() {
        // a pose in the middle of the envelope
        PPRRRKinematics k = new PPRRRKinematics(1, 1);
        PPRRRConfig q = new PPRRRConfig(
                2, 3, 0, -3 * Math.PI / 4, Math.PI / 2);
        Pose3d x = k.forward(q);
        // note displacement
        assertEquals(2, x.getX(), 1e-6);
        assertEquals(3, x.getY(), 1e-6);
        assertEquals(1.414213, x.getZ(), 1e-6);
        assertEquals(0, x.getRotation().getX(), 1e-6);
        assertEquals(-0.785398, x.getRotation().getY(), 1e-6);
        assertEquals(0, x.getRotation().getZ(), 1e-6);
    }

    @Test
    void testInverse0() {
        // in the middle of the field, arm straight out, but above
        // the floor
        PPRRRKinematics k = new PPRRRKinematics(1, 1);
        Pose3d x = new Pose3d(8, 4, 1, new Rotation3d(0, 0, 0));
        PPRRRConfig q = k.inverse(x);
        assertEquals(7, q.q1(), 7e-2);
        assertEquals(4, q.q2(), 5e-3);
        assertEquals(0, q.q3(), 5e-3);
        assertEquals(-1.562, q.q4(), 6e-2);
        assertEquals(1.562, q.q5(), 7e-2);
    }

    @Test
    void testInverse1() {
        // in the middle of the field, arm up, to avoid
        // the redundancy. also avoid the singularities at the
        // edge. The shoulder angle is 150 degrees, so the
        // arm is reaching "back" and the base should be "ahead"
        // of the target.
        PPRRRKinematics k = new PPRRRKinematics(1, 1);
        Pose3d x = new Pose3d(8, 4, 1.5, new Rotation3d(0, -Math.PI / 2, 0));
        // the solver tolerance is low to speed it up.
        // with tighter tolerance, 40 ms. now 27 ms.
        PPRRRConfig q = k.inverse(x);
        System.out.printf("q %s\n", q);
        assertEquals(8.866, q.q1(), 5e-2);
        assertEquals(4, q.q2(), 5e-2);
        assertEquals(0, q.q3(), 5e-2);
        assertEquals(-2.618, q.q4(), 5e-2);
        assertEquals(1.047, q.q5(), 5e-2);
        // moving at a medium speed, new x is 5 cm away,
        // tilted a bit less.
        x = x.plus(new Transform3d(0.05, 0, 0,
                new Rotation3d(0, 0.05, 0)));
        // new solve is 5 ms, still slow but maybe ok
        q = k.inverse(x, q.toVector());
        System.out.printf("q %s\n", q);
    }

    @Test
    void testInverse2() {
        // in the middle of the field, hand on the floor,
        // 30 degrees inclined.
        PPRRRKinematics k = new PPRRRKinematics(1, 1);
        Pose3d x = new Pose3d(8, 4, 0, new Rotation3d(0, 0.523599, 0));
        PPRRRConfig q = k.inverse(x);
        assertEquals(6.268, q.q1(), 5e-2);
        assertEquals(4, q.q2(), 5e-2);
        assertEquals(0, q.q3(), 5e-2);
        assertEquals(-0.523599, q.q4(), 6e-2);
        assertEquals(1.047, q.q5(), 7e-2);
    }
}
