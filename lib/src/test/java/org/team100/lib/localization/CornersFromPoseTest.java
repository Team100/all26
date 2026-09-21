package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

public class CornersFromPoseTest {

    @Test
    void test0() {
        Camera camera = Camera.SIM0;
        CornersFromPose c = new CornersFromPose();
        double[] pts = c.corners(
                camera, new Transform3d(0, 0, 1, new Rotation3d(0, 1, 0)));
        assertArrayEquals(new double[] {
                511, 622,
                594, 632,
                594, 467,
                511, 477 }, pts, 1.0);
    }

    @Test
    void test1() {
        double TAG_SIZE_M = 0.1651;
        double h = TAG_SIZE_M / 2;
        // tag in camera is rotated a bit.
        // note the names for the rotation arguments are wrong. these
        // are the rotation components around the x, y, and z axes,
        // but in this case those axes are not in the usual wpi orientation.
        // so rotating around y is "yaw"
        Transform3d tag = new Transform3d(0, 0, 1, new Rotation3d(0, 1, 0));
        Transform3d corner0 = new Transform3d(-h, h, 0, Rotation3d.kZero);
        Transform3d corner1 = new Transform3d(h, h, 0, Rotation3d.kZero);
        Transform3d corner2 = new Transform3d(h, -h, 0, Rotation3d.kZero);
        Transform3d corner3 = new Transform3d(-h, -h, 0, Rotation3d.kZero);
        Transform3d c0 = tag.plus(corner0);
        Transform3d c1 = tag.plus(corner1);
        Transform3d c2 = tag.plus(corner2);
        Transform3d c3 = tag.plus(corner3);
        TestUtil.verify(new Transform3d(-0.045, 0.083, 1.069, new Rotation3d(0, 1, 0)), c0);
        TestUtil.verify(new Transform3d(0.045, 0.083, 0.931, new Rotation3d(0, 1, 0)), c1);
        TestUtil.verify(new Transform3d(0.045, -0.083, 0.931, new Rotation3d(0, 1, 0)), c2);
        TestUtil.verify(new Transform3d(-0.045, -0.083, 1.069, new Rotation3d(0, 1, 0)), c3);

    }
}
