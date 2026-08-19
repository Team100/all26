package org.team100.lib.kinematics.lynx_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.util.StrUtil;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

public class LynxArmKinematicsTest {
    private static final boolean DEBUG = false;

    @Test
    void testFix1() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(0, Math.PI / 2, 0));
        Pose3d fixed = LynxArmKinematics.fix(p);
        // vertical axis, no effect.
        TestUtil.verify(p, fixed);
    }

    @Test
    void testFix2() {
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d());
        Pose3d fixed = LynxArmKinematics.fix(p);
        // non-vertical axis but the position is ok.
        TestUtil.verify(p, fixed);
    }

    @Test
    void testFix3() {
        // arbitrary rotation
        Pose3d p = new Pose3d(1, 0, 0, new Rotation3d(1, 1, 1));
        Pose3d fixed = LynxArmKinematics.fix(p);
        // non-vertical axis that needs fixing: make yaw zero
        if (DEBUG)
            System.out.printf("fixed %s\n", StrUtil.poseStr(fixed));
        // projection method
        TestUtil.verify(new Pose3d(1, 0, 0, new Rotation3d(0.081, 1.237, 0)), fixed);
        // yaw substitution method
        // TestUtil.verify(new Pose3d(1, 0, 0, new Rotation3d(1, 1, 0)), fixed, "fix3");
    }

    @Test
    void testFix4() {
        // arbitrary rotation
        Pose3d p = new Pose3d(1, 1, 0, new Rotation3d(1, 1, 1));
        Pose3d fixed = LynxArmKinematics.fix(p);
        // non-vertical axis that needs fixing: make yaw match the translation (45 deg)
        if (DEBUG)
            System.out.printf("fixed %s\n", StrUtil.poseStr(fixed));
        // projection method
        TestUtil.verify(new Pose3d(1, 1, 0, new Rotation3d(0.819, 1.01, Math.PI / 4)), fixed);
        // yaw substitution method
        // TestUtil.verify(new Pose3d(1, 1, 0, new Rotation3d(1, 1, Math.PI / 4)),
        // fixed, "fix3");
    }

    @Test
    void testFix5() {
        // a pure pitch
        Pose3d p = new Pose3d(1, 1, 0, new Rotation3d(0, 1, 0));
        Pose3d fixed = LynxArmKinematics.fix(p);
        // this just rotates the pure pitch in yaw.
        if (DEBUG)
            System.out.printf("fixed %s\n", StrUtil.poseStr(fixed));
        // projection method
        TestUtil.verify(new Pose3d(1, 1, 0, new Rotation3d(0.699, 1.145, Math.PI / 4)), fixed);
        // yaw substitution method
        // TestUtil.verify(new Pose3d(1, 1, 0, new Rotation3d(0, 1, Math.PI / 4)),
        // fixed, "fix3");
    }

    @Test
    void testFix6() {
        NumericLynxArmKinematics k = new NumericLynxArmKinematics();
        // initial is pretty close, so the solver always works.
        LynxArmConfig initial = new LynxArmConfig(1.5, -1.5, 2, 1, 0);
        {
            // almost vertical pitch
            Pose3d p = new Pose3d(0, 0.15, 0, new Rotation3d(0, Math.PI / 2 - 0.01, 0));
            Pose3d fixed = LynxArmKinematics.fix(p);
            if (DEBUG)
                System.out.printf("fixed %s\n", StrUtil.poseStr(fixed));
            // projection
            // it just goes all the way to vertical since it's quite close
            TestUtil.verify(new Pose3d(0, 0.15, 0, new Rotation3d(0, Math.PI / 2, 0)), fixed);
            // yaw substitution
            // pitch is left alone but yaw is very wrong
            // TestUtil.verify(new Pose3d(0, 0.15, 0, new Rotation3d(0, Math.PI / 2 - 0.01,
            // Math.PI / 2)), fixed, "fix6");

            LynxArmConfig q = k.inverse(initial, fixed);
            // projection method
            // note roll is more correct
            TestUtil.verify(
                new LynxArmConfig(Math.PI / 2,
                     -1.641, 2.186, 1.026, 1.571), q);
            // yaw substitution method
            // note zero grip roll axis
            // TestUtil.verify(new LynxArmConfig(Math.PI / 2, -1.510, 2.235, 0.836, 0), q);
        }
        {
            // exactly vertical pitch
            Pose3d p = new Pose3d(0, 0.15, 0, new Rotation3d(0, Math.PI / 2, 0));
            Pose3d fixed = LynxArmKinematics.fix(p);
            // this illustrates the problem.
            if (DEBUG)
                System.out.printf("fixed %s\n", StrUtil.poseStr(fixed));
            TestUtil.verify(new Pose3d(0, 0.15, 0, new Rotation3d(0, Math.PI / 2, 0)), fixed);
            LynxArmConfig q = k.inverse(initial, fixed);
            // projection method
            TestUtil.verify(new LynxArmConfig(Math.PI / 2,
                 -1.641, 2.186, 1.026, 1.571), q);
            // yaw substitution method
            // ***
            // note the large change in grip roll axis
            // ***
            // TestUtil.verify(new LynxArmConfig(Math.PI / 2, -1.503, 2.229, 0.845, 1.571),
            // q);
        }
    }

    @Test
    void testPose() {
        // is yaw the thing to replace?
        // these are extrinsic RPY values
        Pose3d p = new Pose3d(0, 0, 0, new Rotation3d(0.5, 0.5, 0.5));
        // so what you get for Z is just what you put in.
        assertEquals(0.5, p.getRotation().getZ(), 1e-3);
    }
}
