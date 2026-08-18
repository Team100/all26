package org.team100.lib.kinematics.urdf;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N5;

public class URDFAL5DTest {
    @Test
    void testZeroForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d()), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.39437, 0, 0.06731, new Rotation3d()), poses, "wrist_rotate");
        TestUtil.verify(new Pose3d(0.44937, 0, 0.06731, new Rotation3d()), poses, "center_point");
    }

    @Test
    void testBaseForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                // positive = pan left, so extent is +x +y
                "base_pan", Math.PI / 4,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.10327, 0.10327, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.23573, 0.23573, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.27886, 0.27886, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses,
                "wrist_rotate");
        TestUtil.verify(new Pose3d(0.31776, 0.31776, 0.06731, new Rotation3d(0, 0, Math.PI / 4)), poses,
                "center_point");
    }

    @Test
    void testShoulderForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // negative = tilt up
                "shoulder_tilt", -Math.PI / 4,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.10372, 0, 0.17058, new Rotation3d(0, -Math.PI / 4, 0)), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.23573, 0, 0.30304, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.27886, 0, 0.34617, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_rotate");
        TestUtil.verify(new Pose3d(0.31775, 0, 0.38506, new Rotation3d(0, -Math.PI / 4, 0)), poses, "center_point");
    }

    @Test
    void testElbowForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                // elbow tilts only down
                "shoulder_tilt", -Math.PI / 4,
                // so the rest should be horizontal
                "elbow_tilt", Math.PI / 4,
                "wrist_tilt", 0.0,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.10372, 0, 0.17058, new Rotation3d()), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.29060, 0, 0.17058, new Rotation3d()), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.35159, 0, 0.17058, new Rotation3d()), poses, "wrist_rotate");
        TestUtil.verify(new Pose3d(0.40659, 0, 0.17058, new Rotation3d()), poses, "center_point");
    }

    @Test
    void testWristForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                // wrist tilt up a bit
                "wrist_tilt", -Math.PI / 4,
                "wrist_rotate", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, 0)), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.37650, 0, 0.11044, new Rotation3d(0, -Math.PI / 4, 0)), poses, "wrist_rotate");
        TestUtil.verify(new Pose3d(0.41539, 0, 0.14933, new Rotation3d(0, -Math.PI / 4, 0)), poses, "center_point");
    }

    @Test
    void testWristRotateForward() {
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", 0.0,
                "elbow_tilt", 0.0,
                "wrist_tilt", 0.0,
                // rotate clockwise looking down x
                "wrist_rotate", Math.PI / 4);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(6, poses.size());
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "base_pan");
        TestUtil.verify(new Pose3d(0, 0, 0.06731, new Rotation3d()), poses, "shoulder_tilt");
        TestUtil.verify(new Pose3d(0.14605, 0, 0.06731, new Rotation3d()), poses, "elbow_tilt");
        TestUtil.verify(new Pose3d(0.33337, 0, 0.06731, new Rotation3d()), poses, "wrist_tilt");
        TestUtil.verify(new Pose3d(0.39437, 0, 0.06731, new Rotation3d(Math.PI / 4, 0, 0)), poses, "wrist_rotate");
        TestUtil.verify(new Pose3d(0.44937, 0, 0.06731, new Rotation3d(Math.PI / 4, 0, 0)), poses, "center_point");
    }

    @Test
    void testCenterPointForward() {
        // this is what the inverse kinematics below should come up with
        URDFAL5D m = URDFAL5D.make();
        Map<String, Double> q = Map.of(
                "base_pan", 0.0,
                "shoulder_tilt", -2.522,
                "elbow_tilt", 2.804,
                "wrist_tilt", -0.282,
                "wrist_rotate", 0.0,
                "center_point", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        TestUtil.verify(new Pose3d(0.177024, 0, 0.1, new Rotation3d(0, 0, 0)), poses, "center_point");
    }

    @Test
    void testElbowTransform() {
        URDFAL5D m = URDFAL5D.make();
        URDFJoint j = m.getJoint("elbow_tilt");
        Transform3d t = j.transform(Math.PI / 4);
        TestUtil.verify(new Transform3d(0.14605, 0, 0, new Rotation3d(0, Math.PI / 4, 0)), t);
    }

    @Test
    void testCenterTransform() {
        URDFAL5D m = URDFAL5D.make();
        URDFJoint j = m.getJoint("center_point");
        // the parameter here is ignored.
        Transform3d t = j.transform(1.0);
        TestUtil.verify(new Transform3d(0.055, 0, 0, new Rotation3d()), t);
    }

    @Test
    void testInverse() {
        // this problem requires the dx limit, otherwise it oscillates
        // far from the solution.
        URDFAL5D m = URDFAL5D.make();
        Pose3d end = new Pose3d(0.2, 0.0, 0.1, new Rotation3d(0, 0, 0));
        Vector<N5> q0 = VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1);
        Map<String, Double> qMap = m.inverse(
                q0, "center_point", end);
        TestUtil.verify(Map.of(
                "base_pan", 0.000,
                "shoulder_tilt", -2.157,
                "elbow_tilt", 2.652,
                "wrist_tilt", -0.495,
                "wrist_rotate", 0.000), qMap);
    }

    @Test
    void testConvergence() {
        URDFAL5D m = URDFAL5D.make();
        LynxArmConfig c = new LynxArmConfig(
                2.0994465067e-04,
                -1.8609471376e+00,
                1.5635203893e+00,
                1.0872555301e+00,
                1.4888289270e-04);
        Pose3d goal = new Pose3d(
                new Translation3d(0.19991979, 0.0011040928, 0.19832649),
                new Rotation3d(3.3019369e-18, 0.79406969, 7.6530612e-19));
        m.inverse(c.toVec(), "center_point", goal);
    }

}
