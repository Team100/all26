package org.team100.lib.geometry.se2_universal_cover;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

public class WindingPose2dTest {
    @Test
    void test0() {
        // the difference between Pose2d and WindingPose2d is the rotation wrapping.
        Pose2d p0 = new Pose2d();
        Pose2d p1 = p0.plus(new Transform2d(0, 0, new Rotation2d(10)));
        // note the wrapping rotation
        TestUtil.verify(new Pose2d(0, 0, new Rotation2d(-2.566)), p1);

        WindingPose2d t0 = new WindingPose2d();
        WindingPose2d t1 = t0.plus(new WindingTransform2d(0, 0, 10));
        // note the absence of wrapping
        TestUtil.verify(new WindingPose2d(0, 0, 10), t1);
    }

}
