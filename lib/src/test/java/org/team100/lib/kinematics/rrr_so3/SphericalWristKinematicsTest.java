package org.team100.lib.kinematics.rrr_so3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.geometry.six_dof.SphericalWristPose;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.geometry.Rotation3d;

public class SphericalWristKinematicsTest {

    @Test
    void testForward1() {
        // all zero: rot is identity
        SphericalWristKinematics wk = new SphericalWristKinematics();
        SphericalWristConfig q = new SphericalWristConfig(0, 0, 0);
        SphericalWristPose P = wk.forward(q);
        TestUtil.verify(new Rotation3d(), P.p4().getRotation());
        TestUtil.verify(new Rotation3d(), wk.forward(q).p5().getRotation());
        TestUtil.verify(new Rotation3d(), wk.forward(q).p6().getRotation());
    }

    @Test
    void testForward2() {
        // pitch up
        SphericalWristKinematics wk = new SphericalWristKinematics();
        SphericalWristConfig q = new SphericalWristConfig(0, Math.PI / 2, 0);
        SphericalWristPose P = wk.forward(q);
        TestUtil.verify(new Rotation3d(0, 0, 0), P.p4().getRotation());
        TestUtil.verify(new Rotation3d(0, -Math.PI / 2, 0), P.p5().getRotation());
        TestUtil.verify(new Rotation3d(0, -Math.PI / 2, 0), P.p6().getRotation());
    }

    @Test
    void testForward3() {
        // roll 180 and then pitch down, then roll again -> pitch up
        SphericalWristKinematics wk = new SphericalWristKinematics();
        SphericalWristConfig q = new SphericalWristConfig(Math.PI, -Math.PI / 2, Math.PI);
        SphericalWristPose p = wk.forward(q);
        TestUtil.verify(new Rotation3d(Math.PI, 0, 0), p.p4().getRotation());
        TestUtil.verify(new Rotation3d(0, -Math.PI / 2, Math.PI), p.p5().getRotation());
        TestUtil.verify(new Rotation3d(0, -Math.PI / 2, 0), p.p6().getRotation());
    }

    @Test
    void testInverse1() {
        // identity => singularity, throws because default is null.
        SphericalWristKinematics wk = new SphericalWristKinematics();
        Rotation3d r = new Rotation3d();
        assertThrows(IllegalArgumentException.class, () -> wk.inverse(r, null));
    }

    @Test
    void testInverse1a() {
        // identity => singularity, uses default, roll sum is zero.
        SphericalWristKinematics wk = new SphericalWristKinematics();
        Rotation3d r = new Rotation3d();
        List<SphericalWristConfig> q = wk.inverse(r, 1.0);
        assertEquals(1, q.size());
        assertEquals(1, q.get(0).q4(), 1e-3);
        assertEquals(0, q.get(0).q5(), 1e-3);
        assertEquals(-1, q.get(0).q6(), 1e-3);
    }

    @Test
    void testInverse1b() {
        // roll only => singularity, uses default, roll sum is zero.
        SphericalWristKinematics wk = new SphericalWristKinematics();
        // "Roll" in the wrist frame is around x
        Rotation3d r = new Rotation3d(1, 0, 0);
        List<SphericalWristConfig> q = wk.inverse(r, 1.0);
        assertEquals(1, q.size());
        assertEquals(1, q.get(0).q4(), 1e-3);
        assertEquals(0, q.get(0).q5(), 1e-3);
        assertEquals(0, q.get(0).q6(), 1e-3);
    }

    @Test
    void testInverse2() {
        // point up
        SphericalWristKinematics wk = new SphericalWristKinematics();
        Rotation3d r = new Rotation3d(0, -Math.PI / 2, 0);
        List<SphericalWristConfig> q = wk.inverse(r, 0.0);
        assertEquals(2, q.size());
        // note two opposite cases
        TestUtil.verify(new SphericalWristConfig(3.141, -1.571, -3.141), q.get(0));
        TestUtil.verify(new SphericalWristConfig(0, 1.571, 0), q.get(1));
    }
}
