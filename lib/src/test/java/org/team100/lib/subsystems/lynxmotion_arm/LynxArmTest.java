package org.team100.lib.subsystems.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.kinematics.lynx_arm.AnalyticLynxArmKinematics;
import org.team100.lib.kinematics.lynx_arm.LynxArmKinematics;
import org.team100.lib.kinematics.lynx_arm.NumericLynxArmKinematics;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmTest {
    private static final boolean DEBUG = false;

    @Test
    void testTwist() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d start = new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(start);
            Pose3d end = new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
            if (DEBUG) {
                System.out.printf("start %s\n", StrUtil.poseStr(start));
                System.out.printf("end %s\n", StrUtil.poseStr(end));
            }
            for (double s = 0; s <= 1; s += 0.2) {
                Pose3d lerp = LynxArmPose.interpolate(start, end, s);
                m_arm.setPosition(lerp);
                // wrist should be pointing down the whole time
                LynxArmPose p = m_arm.getPosition();
                if (DEBUG) {
                    System.out.printf("s %f p1 %s\n", s, StrUtil.poseStr(p.p1()));
                    System.out.printf("s %f p2 %s\n", s, StrUtil.poseStr(p.p2()));
                    System.out.printf("s %f p3 %s\n", s, StrUtil.poseStr(p.p3()));
                    System.out.printf("s %f p4 %s\n", s, StrUtil.poseStr(p.p4()));
                    System.out.printf("s %f p5 %s\n", s, StrUtil.poseStr(p.p5()));
                    System.out.printf("s %f p6 %s\n", s, StrUtil.poseStr(p.p6()));
                }
            }
        }
    }

    @Test
    void testRoundTrip() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.15, 0.1, 0),
                    new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", StrUtil.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTripb() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = new NumericLynxArmKinematics();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.15, 0.1, 0),
                    new Rotation3d(0, Math.PI / 2, 0));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", StrUtil.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testRoundTrip2() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            Pose3d setpoint = new Pose3d(
                    new Translation3d(0.165326, 0.056973, 0.085628),
                    new Rotation3d(0.172187, 0.923138, -0.154456));
            m_arm.setPosition(setpoint);
            if (DEBUG)
                System.out.printf("setpoint %s\n", StrUtil.poseStr(setpoint));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("measured  config %s\n", measuredConfig);
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("commanded config %s\n", commandedConfig);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
        }
    }

    @Test
    void testHome() {
        // numeric kinematics
        final Pose3d HOME = new Pose3d(0.2, 0, 0.2, new Rotation3d(0, Math.PI / 4, 0));
        final LynxArmKinematics kinematics = new NumericLynxArmKinematics();
        LynxArmConfig initial = new LynxArmConfig(0, 0, 0, 0, 0);
        LynxArmConfig q = kinematics.inverse(initial, HOME);
        // zero swing, pitch up, elbow and wrist down, no twist.
        TestUtil.verify(q, 0.00, -1.936, 1.505, 1.217, 0.00);
    }

    @Test
    void testHome2() {
        final LynxArmKinematics kinematics = new NumericLynxArmKinematics();
        try (LynxArm m_arm = new LynxArm(kinematics)) {
            LynxArmConfig q = m_arm.getMeasuredConfig();
            TestUtil.verify(q, 0.00, -1.939, 1.507, 1.216, 0.00);
        }
    }

    @Test
    void testHome3() {
        // analytic kinematics
        final Pose3d HOME = new Pose3d(0.2, 0, 0.2, new Rotation3d(0, Math.PI / 4, 0));
        final LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();
        LynxArmConfig initial = new LynxArmConfig(0, 0, 0, 0, 0);
        LynxArmConfig q = kinematics.inverse(initial, HOME);
        // zero swing, pitch up, elbow and wrist down, no twist.
        TestUtil.verify(q, 0.00, -1.936, 1.505, 1.217, 0.00);
    }

    @Test
    void testHome4() {
        final LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();
        try (LynxArm m_arm = new LynxArm(kinematics)) {
            LynxArmConfig q = m_arm.getMeasuredConfig();
            TestUtil.verify(q, 0.00, -1.936, 1.505, 1.217, 0.00);
        }
    }

    @Test
    void testRoundTrip2b() {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        final LynxArmKinematics kinematics = new NumericLynxArmKinematics();

        try (LynxArm m_arm = new LynxArm(kinematics)) {
            // first check the home position
            LynxArmConfig q = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("** INITIAL %s\n", q.str());
            // there's something wrong with this setpoint.
            // Pose3d setpoint = new Pose3d(
            // new Translation3d(0.165326, 0.056973, 0.085628),
            // new Rotation3d(0.172187, 0.923138, -0.154456));
            final Pose3d setpoint = new Pose3d(
                    new Translation3d(0.2, 0.0, 0.2),
                    new Rotation3d(0.0, 0.0, 0.0));

            if (DEBUG) {
                System.out.printf("** SETPOINT %s\n", StrUtil.poseStr(setpoint));
                LynxArmConfig inv = kinematics.inverse(new LynxArmConfig(0, 0, 0, 0, 0), setpoint);
                System.out.printf("** INV %s\n", inv.str());
            }

            m_arm.setPosition(setpoint);

            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            if (DEBUG)
                System.out.printf("** MEASURED %s\n", measuredConfig.str());

            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            if (DEBUG)
                System.out.printf("** INV2 %s\n", commandedConfig.str());

            assertEquals(measuredConfig.swing().getAsDouble(), commandedConfig.swing().getAsDouble(), 0.001);
            assertEquals(measuredConfig.boom(), commandedConfig.boom(), 0.001);
            assertEquals(measuredConfig.stick(), commandedConfig.stick(), 0.001);
            assertEquals(measuredConfig.wrist(), commandedConfig.wrist(), 0.001);
            assertEquals(measuredConfig.twist().getAsDouble(), commandedConfig.twist().getAsDouble(), 0.001);
        }
    }
}
