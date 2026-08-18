package org.team100.lib.kinematics.lynx_arm;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class NumericLynxArmKinematicsTest {
    @Test
    void testConvergence() {
        NumericLynxArmKinematics k = new NumericLynxArmKinematics();
        LynxArmConfig c = new LynxArmConfig(
                2.0994465067e-04,
                -1.8609471376e+00,
                1.5635203893e+00,
                1.0872555301e+00,
                1.4888289270e-04);
        Pose3d goal = new Pose3d(
                new Translation3d(0.19991979, 0.0011040928, 0.19832649),
                new Rotation3d(3.3019369e-18, 0.79406969, 7.6530612e-19));
        @SuppressWarnings("unused")
        LynxArmConfig i = k.inverse(c, goal);
    }
}
