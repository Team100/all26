package org.team100.lib.kinematics.lynx_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.testing.TestUtil;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class AnalyticLynxArmKinematicsTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void testTwist() {
        // figure out the twist axis
        Rotation3d r = new Rotation3d(Math.PI / 2, Math.PI / 2, Math.PI / 2);
        if (DEBUG)
            System.out.printf("r %s\n", StrUtil.rotStr(r));

    }

    @Test
    void testDerived() {
        // derive the axis directions
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.real();
        LynxArmConfig q0 = new LynxArmConfig(0, 0, 0, 0, 0);
        Pose3d end = new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
        LynxArmConfig q = k.inverse(q0, end);
        if (DEBUG)
            System.out.printf("q %s\n", q);
        LynxArmPose p = k.forward(q);
        if (DEBUG) {
            System.out.printf("p1 %s\n", StrUtil.poseStr(p.p1()));
            System.out.printf("p2 %s\n", StrUtil.poseStr(p.p2()));
            System.out.printf("p3 %s\n", StrUtil.poseStr(p.p3()));
            System.out.printf("p4 %s\n", StrUtil.poseStr(p.p4()));
            System.out.printf("p5 %s\n", StrUtil.poseStr(p.p5()));
            System.out.printf("p6 %s\n", StrUtil.poseStr(p.p6()));
        }
        // the difference in joint poses here produces a pure pitch
        // which does not match my intuition (that the joint axis would be rotated)
        Rotation3d r = p.p3().getRotation().minus(p.p2().getRotation());
        if (DEBUG)
            System.out.printf("r %s\n", StrUtil.rotStr(r));
        Translation3d t = new Translation3d(1, 0, 0);
        Vector<N3> v2 = t.rotateBy(p.p2().getRotation()).toVector();
        Vector<N3> v3 = t.rotateBy(p.p3().getRotation()).toVector();
        Vector<N3> axis = Vector.cross(v2, v3);
        Rotation3d axisR = new Rotation3d(axis);
        if (DEBUG)
            System.out.printf("axisR %s\n", StrUtil.rotStr(axisR));
        Translation3d tR = new Translation3d(axis);
        if (DEBUG)
            System.out.printf("tR %s\n", tR);

        // this way is simpler
        Translation3d yt = new Translation3d(0, 1, 0);
        Translation3d a2 = new Translation3d(yt.rotateBy(p.p3().getRotation()).toVector());
        if (DEBUG)
            System.out.printf("a2 %s\n", a2);

    }

    @Test
    void testWrist() {
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.real();
        Pose3d start = new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
        Pose3d end = new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0));
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        for (double s = 0; s <= 1; s += 0.1) {
            Pose3d lerp = LynxArmPose.interpolate(start, end, s);
            // wrist should be pointing down the whole time
            q = k.inverse(q, lerp);
            if (DEBUG)
                System.out.printf("q %s\n", StrUtil.poseStr(lerp));
        }
    }

    @Test
    void testWrist2() {
        // is the wrist logic right?
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.real();

        // this is from an error case
        Pose3d p = new Pose3d(
                new Translation3d(0.191993, 0.013100, 0.092421),
                new Rotation3d(0.051100, 0.220814, -0.048971));

        LynxArmConfig q0 = new LynxArmConfig(0, 0, 0, 0, 0);

        // this does indeed produce an error
        @SuppressWarnings("unused")
        LynxArmConfig q = k.inverse(q0, p);

        Translation2d translation = p.toPose2d().getTranslation();
        Rotation3d endRotation = p.getRotation();

        Rotation2d swingAngle = translation.getAngle();
        if (DEBUG)
            System.out.printf("swing angle rad %f\n", swingAngle.getRadians());
        Rotation3d swing3d = new Rotation3d(swingAngle);
        Rotation3d swingRelative3d = endRotation.minus(swing3d);
        if (DEBUG)
            System.out.printf("swing relative %s\n", swingRelative3d);

    }

    @Test
    void testBadRotation() {
        // stretched out along x but with the wrong end rotation
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose t = new LynxArmPose(
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(new Translation3d(2, 0, 1), new Rotation3d(0, 0, 1)));
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        LynxArmConfig foo = k.inverse(q, t.p6());
        assertEquals(0, foo.swing().getAsDouble(), DELTA);
        assertEquals(-0.288, foo.boom(), DELTA);
        assertEquals(0.576, foo.stick(), DELTA);
        assertEquals(-0.288, foo.wrist(), DELTA);
        assertEquals(0, foo.twist().getAsDouble(), DELTA);
    }

    @Test
    void testOutOfBounds() {
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose t = new LynxArmPose(
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(new Translation3d(5, 0, 1), new Rotation3d(0, 0, 0)));
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);

        assertThrows(IllegalArgumentException.class, () -> k.inverse(q, t.p6()));
    }

    @Test
    void test1() {
        // stretched out along x
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(1, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(2, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(3, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(4, 0, 1), new Rotation3d()));
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test2() {
        // stretched out along x
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.half();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 0.5), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 0.5), new Rotation3d()),
                new Pose3d(new Translation3d(0.5, 0, 0.5), new Rotation3d()),
                new Pose3d(new Translation3d(1.0, 0, 0.5), new Rotation3d()),
                new Pose3d(new Translation3d(1.5, 0, 0.5), new Rotation3d()),
                new Pose3d(new Translation3d(2.0, 0, 0.5), new Rotation3d()));
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test3() {
        // up at the shoulder, forward at the elbow
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.PI / 2, 0)),
                new Pose3d(new Translation3d(0, 0, 2), new Rotation3d()),
                new Pose3d(new Translation3d(1, 0, 2), new Rotation3d()),
                new Pose3d(new Translation3d(2, 0, 2), new Rotation3d()),
                new Pose3d(new Translation3d(3, 0, 2), new Rotation3d()));
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 2, Math.PI / 2, 0, 0);
        verify(k, p, q);
    }

    @Test
    void test4() {
        // boom and stick make an equilateral, wrist points ahead
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.PI / 3, 0)),
                new Pose3d(new Translation3d(0.5, 0, 1 + Math.sqrt(3) / 2), new Rotation3d(0, Math.PI / 3, 0)),
                new Pose3d(new Translation3d(1, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(2, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(3, 0, 1), new Rotation3d()));
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, -Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void test5() {
        // boom and stick make an equilateral, wrist points ahead, with swing
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, Math.PI / 6)),
                new Pose3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, -Math.PI / 3, Math.PI / 6)),
                new Pose3d(
                        new Translation3d(Math.sqrt(3) / 4, 0.25, 1 + Math.sqrt(3) / 2),
                        new Rotation3d(0, Math.PI / 3, Math.PI / 6)),
                new Pose3d(
                        new Translation3d(Math.sqrt(3) / 2, 0.5, 1),
                        new Rotation3d(0, 0, Math.PI / 6)),
                new Pose3d(
                        new Translation3d(Math.sqrt(3), 1, 1),
                        new Rotation3d(0, 0, Math.PI / 6)),
                new Pose3d(
                        new Translation3d(1.5 * Math.sqrt(3), 1.5, 1),
                        new Rotation3d(0, 0, Math.PI / 6)));
        LynxArmConfig q = new LynxArmConfig(Math.PI / 6, -Math.PI / 3, 2 * Math.PI / 3, -Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void test6() {
        // wrist vertical
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, 0);
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.PI / 3, 0)),
                new Pose3d(new Translation3d(0.5, 0, 1 + Math.sqrt(3) / 2), new Rotation3d(0, Math.PI / 3, 0)),
                new Pose3d(new Translation3d(1, 0, 1), new Rotation3d(0, Math.PI / 2, 0)),
                new Pose3d(new Translation3d(1, 0, 0), new Rotation3d(0, Math.PI / 2, 0)),
                new Pose3d(new Translation3d(1, 0, -1), new Rotation3d(0, Math.PI / 2, 0)));
        verify(k, p, q);
    }

    @Test
    void test7() {
        // wrist vertical with roll (which is yaw but inverted since it's pointing down)
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, Math.PI / 2);
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d(0, -Math.PI / 3, 0)),
                new Pose3d(new Translation3d(0.5, 0, 1 + Math.sqrt(3) / 2), new Rotation3d(0, Math.PI / 3, 0)),
                new Pose3d(new Translation3d(1, 0, 1), new Rotation3d(0, Math.PI / 2, 0)),
                new Pose3d(new Translation3d(1, 0, 0), new Rotation3d(0, Math.PI / 2, -Math.PI / 2)),
                new Pose3d(new Translation3d(1, 0, -1), new Rotation3d(0, Math.PI / 2, -Math.PI / 2)));
        verify(k, p, q);
    }

    @Test
    void test8() {
        // wrist vertical with roll and swing
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmConfig q = new LynxArmConfig(
                Math.PI / 3, -Math.PI / 3, 2 * Math.PI / 3, Math.PI / 6, Math.PI / 2);
        LynxArmPose p = new LynxArmPose(
                new Pose3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, Math.PI / 3)),
                new Pose3d(
                        new Translation3d(0., 0, 1),
                        new Rotation3d(0, -Math.PI / 3, Math.PI / 3)),
                new Pose3d(
                        new Translation3d(0.25, Math.sqrt(3) / 4, 1 + Math.sqrt(3) / 2),
                        new Rotation3d(0, Math.PI / 3, Math.PI / 3)),
                new Pose3d(
                        new Translation3d(0.5, Math.sqrt(3) / 2, 1),
                        new Rotation3d(0, Math.PI / 2, Math.PI / 3)),
                new Pose3d(
                        new Translation3d(0.5, Math.sqrt(3) / 2, 0),
                        new Rotation3d(0, Math.PI / 2, -Math.PI / 6)),
                new Pose3d(
                        new Translation3d(0.5, Math.sqrt(3) / 2, -1),
                        new Rotation3d(0, Math.PI / 2, -Math.PI / 6)));
        verify(k, p, q);
    }

    @Test
    void test9() {
        // grip is on the swing axis, make the swing match end yaw
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, 0, -Math.PI / 4)),
                new Pose3d(
                        new Translation3d(0, 0, 1),
                        new Rotation3d(0, -Math.PI, -Math.PI / 4)),
                new Pose3d(
                        new Translation3d(-Math.sqrt(2) / 2, Math.sqrt(2) / 2, 1),
                        new Rotation3d(0, -Math.PI / 2, -Math.PI / 4)),
                new Pose3d(
                        new Translation3d(-Math.sqrt(2) / 2, Math.sqrt(2) / 2, 2),
                        new Rotation3d(0, 0, -Math.PI / 4)),
                new Pose3d(
                        new Translation3d(0, 0, 2),
                        new Rotation3d(0, 0, -Math.PI / 4)),
                new Pose3d(
                        new Translation3d(Math.sqrt(2) / 2, -Math.sqrt(2) / 2, 2),
                        new Rotation3d(0, 0, -Math.PI / 4)));
        LynxArmConfig q = new LynxArmConfig(-Math.PI / 4, -Math.PI, Math.PI / 2, Math.PI / 2, 0);
        verify(k, p, q);
    }

    @Test
    void test10() {
        // grip is on the swing axis, wrist pointing up => indeterminate
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d(0, -5 * Math.PI / 6, 0)),
                new Pose3d(new Translation3d(-Math.sqrt(3) / 2, 0, 1.5), new Rotation3d(0, -Math.PI / 6, 0)),
                new Pose3d(new Translation3d(0, 0, 2), new Rotation3d(0, -Math.PI / 2, 0)),
                new Pose3d(new Translation3d(0, 0, 3), new Rotation3d(0, -Math.PI / 2, 0)),
                new Pose3d(new Translation3d(0, 0, 4), new Rotation3d(0, -Math.PI / 2, 0)));
        LynxArmConfig q = new LynxArmConfig(
                OptionalDouble.empty(), -5 * Math.PI / 6, 2 * Math.PI / 3, -Math.PI / 3, OptionalDouble.empty());
        verify(k, p, q);
    }

    @Test
    void test11() {
        // arch
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        LynxArmPose p = new LynxArmPose(
                new Pose3d(new Translation3d(0, 0, 1), new Rotation3d()),
                new Pose3d(new Translation3d(0.0, 0, 1), new Rotation3d(0, -Math.PI / 3, 0)),
                new Pose3d(new Translation3d(0.5, 0, 1 + Math.sqrt(3) / 2), new Rotation3d()),
                new Pose3d(new Translation3d(1.5, 0, 1 + Math.sqrt(3) / 2), new Rotation3d(0, Math.PI / 3, 0)),
                new Pose3d(new Translation3d(2, 0, 1), new Rotation3d(0, Math.PI / 3, 0)),
                new Pose3d(new Translation3d(2.5, 0, 1 - Math.sqrt(3) / 2), new Rotation3d(0, Math.PI / 3, 0)));
        LynxArmConfig q = new LynxArmConfig(0, -Math.PI / 3, Math.PI / 3, Math.PI / 3, 0);
        verify(k, p, q);
    }

    @Test
    void testPath() {
        // print the path from (1,1,0) to (1,-1,0). end effector on the table, wrist
        // vertical, no workspace twist
        AnalyticLynxArmKinematics k = AnalyticLynxArmKinematics.unit();
        Translation3d start = new Translation3d(1, 1, 0);
        Translation3d end = new Translation3d(1, -1, 0);
        LynxArmConfig q = new LynxArmConfig(0, 0, 0, 0, 0);
        // end-effector rotation is fixed
        Rotation3d r = new Rotation3d(0, Math.PI / 2, 0);
        if (DEBUG)
            System.out.println("s, swing, boom, stick, wrist, twist");
        for (double s = 0; s <= 1.0; s += 0.05) {
            Translation3d t = start.interpolate(end, s);
            Pose3d p = new Pose3d(t, r);
            q = k.inverse(q, p);
            if (DEBUG)
                System.out.printf("%6.3f, %s\n", s, q.str());
        }
    }

    void verify(AnalyticLynxArmKinematics k, LynxArmPose p, LynxArmConfig q) {
        TestUtil.verify(p, k.forward(q));
        LynxArmConfig q0 = new LynxArmConfig(0, 0, 0, 0, 0);
        verifyInv(q, k.inverse(q0, p.p6()));
    }

    void verifyInv(LynxArmConfig expected, LynxArmConfig actual) {
        if (expected.swing().isPresent()) {
            assertEquals(MathUtil.angleModulus(expected.swing().getAsDouble()),
                    MathUtil.angleModulus(actual.swing().getAsDouble()), DELTA, "inv swing");
        } else {
            assertTrue(actual.swing().isEmpty(), "inv swing");
        }
        assertEquals(MathUtil.angleModulus(expected.boom()), MathUtil.angleModulus(actual.boom()), DELTA, "inv boom");
        assertEquals(MathUtil.angleModulus(expected.stick()), MathUtil.angleModulus(actual.stick()), DELTA,
                "inv stick");
        assertEquals(MathUtil.angleModulus(expected.wrist()), MathUtil.angleModulus(actual.wrist()), DELTA,
                "inv wrist");
        if (expected.twist().isPresent()) {
            assertEquals(MathUtil.angleModulus(expected.twist().getAsDouble()),
                    MathUtil.angleModulus(actual.twist().getAsDouble()), DELTA, "inv twist");
        } else {
            assertTrue(actual.twist().isEmpty());
        }
    }
}