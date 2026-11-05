package org.team100.lib.localization;

import org.junit.jupiter.api.Test;
import org.team100.lib.camera.Camera;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PoseFromCornersTest {
    @Test
    void test0() {
        PoseFromCorners estimator = new PoseFromCorners();
        Camera camera = Camera.SIM0;
        double[] corners = { 500, 600, 600, 600, 600, 500, 500, 500 };
        Transform3d zfwd = estimator.pose(camera, corners);
        TestUtil.verify(new Transform3d(0, 0, 1.543, Rotation3d.kZero), zfwd);
    }

}
