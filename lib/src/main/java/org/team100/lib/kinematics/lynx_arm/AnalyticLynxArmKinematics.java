package org.team100.lib.kinematics.lynx_arm;

import java.util.List;
import java.util.OptionalDouble;

import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;


public class AnalyticLynxArmKinematics implements LynxArmKinematics {
    private final double m_swingHeight;
    private final double m_boomLength;
    private final double m_stickLength;
    private final double m_wristLength;
    private final double m_gripLength;
    private final RRKinematics twodof;

    /** All distances are in meters. */
    private AnalyticLynxArmKinematics(
            double swingHeight,
            double boomLength,
            double stickLength,
            double wristLength,
            double gripLength) {
        m_swingHeight = swingHeight;
        m_boomLength = boomLength;
        m_stickLength = stickLength;
        m_wristLength = wristLength;
        m_gripLength = gripLength;
        twodof = new RRKinematics(boomLength, stickLength);
    }

    public static AnalyticLynxArmKinematics unit() {
        return new AnalyticLynxArmKinematics(1, 1, 1, 1, 1);
    }

    public static AnalyticLynxArmKinematics half() {
        return new AnalyticLynxArmKinematics(0.5, 0.5, 0.5, 0.5, 0.5);
    }

    public static AnalyticLynxArmKinematics real() {
        return new AnalyticLynxArmKinematics(0.06731, 0.14605, 0.187325, 0.061, 0.055);
    }

    /**
     * Applies the configuration transforms in order to obtain the end-effector
     * (grip center) pose relative to the arm origin, which is the on the tabletop
     * at the swing axis.
     */
    @Override
    public LynxArmPose forward(LynxArmConfig joints) {
        // these names mirror the URDF AL5D names.
        // first translate and then rotate
        Pose3d root = Pose3d.kZero;
        // the end of the swing axis
        Pose3d base_pan = root.transformBy(z(m_swingHeight)).transformBy(joints.swingT());
        // shoulder "link" has zero length, so this pose is still on the yaw axis
        Pose3d shoulder_tilt = base_pan.transformBy(joints.boomT());
        // the pose of the elbow, including the elbow rotation
        Pose3d elbow_tilt = shoulder_tilt.transformBy(x(m_boomLength)).transformBy(joints.stickT());
        // the pose of the wrist, including the wrist rotation
        Pose3d wrist_tilt = elbow_tilt.transformBy(x(m_stickLength)).transformBy(joints.wristT());
        // the pose at the twisting part, i.e. between the twisting servo and the grip
        // servo
        Pose3d wrist_rotate = wrist_tilt.transformBy(x(m_wristLength)).transformBy(joints.twistT());
        // center point is just extension
        Pose3d center_point = wrist_rotate.transformBy(x(m_gripLength));
        return new LynxArmPose(
                base_pan,
                shoulder_tilt,
                elbow_tilt,
                wrist_tilt,
                wrist_rotate,
                center_point);

        // // at the boom joint
        // //
        // Pose3d boomRoot =
        // root.transformBy(joints.swingT()).transformBy(z(m_swingHeight));
        // // the end of the boom, at the stick joint
        // Pose3d stickRoot =
        // boomRoot.transformBy(joints.boomT()).transformBy(x(m_boomLength));
        // // the end of the stick, at the wrist joint
        // Pose3d wristRoot =
        // stickRoot.transformBy(joints.stickT()).transformBy(x(m_stickLength));
        // // this is actually at the grip center but without the twist,
        // // since the twist axis and the grip axis are collinear.
        // Pose3d twistRoot =
        // wristRoot.transformBy(joints.wristT()).transformBy(x(m_wristLength));
        // // this is Tool Center Point.
        // Pose3d grip = twistRoot.transformBy(joints.twistT());
        // return new LynxArmPose(boomRoot, stickRoot, wristRoot, twistRoot, grip);
    }

    /**
     * Solve the inverse kinematics for the given end-effector pose.
     * 
     * Ignores the initial config.
     * 
     * Refer to the diagram
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     * 
     * TODO: this includes fixups for the bad Rotation2d winding behavior which is
     * fixed in 2027
     */
    @Override
    public LynxArmConfig inverse(LynxArmConfig c, final Pose3d end) {

        final OptionalDouble swing;
        final double boom;
        final double stick;
        final double wrist;
        final OptionalDouble twist;

        Translation2d translation = end.toPose2d().getTranslation();

        // To find the wrist joint location, follow the wrist backwards.
        Pose3d wristPose = end.transformBy(x(m_wristLength + m_gripLength).inverse());

        // Solve the 2dof problem. The 2dof coordinates are different:
        // the x dimension is the radius in 3d xy.
        // the swing is limited to +/- pi/2, so if the 3d "x" value is negative (i.e.
        // quadrant 3 or 4) then the 2d x value should be too.
        double hypot = Math.hypot(wristPose.getX(), wristPose.getY()) * Math.signum(wristPose.getX());
        // 3d z becomes y, offset by the swing height.
        double twoDofY = wristPose.getZ() - m_swingHeight;
        Translation2d twoDofEnd = new Translation2d(
                hypot,
                twoDofY);
        // TODO: default
        List<RRConfig> twoDofConfig = twodof.inverse(twoDofEnd, null);
        // for now, throw like it used to
        // TODO: handle zero solutions gracefully
        if (twoDofConfig.isEmpty())
            throw new IllegalArgumentException();

        // the 2d coordinates are inverted for convenience, so fix it here.
        // TODO: assuming the first result is good.
        // TODO: instead, test against joint limits and choose the feasible one.
        boom = -1.0 * twoDofConfig.get(0).q1();
        stick = -1.0 * twoDofConfig.get(0).q2();

        Rotation3d endRotation = end.getRotation();
        if (translation.getNorm() < 1e-3) {
            // System.out.println("The end pose is above the swing axis.");

            if (MathUtil.isNear(-Math.PI / 2, endRotation.getY(), 0.01)) {
                // The wrist axis is pointing up.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = -Math.PI / 2 - boom - stick;
            } else if (MathUtil.isNear(Math.PI / 2, endRotation.getY(), 0.01)) {
                // The wrist axis is pointing down.
                // Both twist and swing are indeterminate; the caller should decide what to do.
                swing = OptionalDouble.empty();
                twist = OptionalDouble.empty();
                wrist = Math.PI / 2 - boom - stick;
            } else {
                // The end is on the swing axis, but the wrist is not, so the arm swing needs to
                // be aligned with the end angle.
                double swingAngle = endRotation.getZ();
                swing = OptionalDouble.of(swingAngle);
                // to get the wrist rotation, project the swing-relative rotation
                Rotation3d swing3d = new Rotation3d(0, 0, swingAngle);
                Rotation3d swingRelative3d = endRotation.rotateBy(swing3d.inverse());
                // to get the stick-relative wrist angle, subtract the joints
                wrist = swingRelative3d.getY() - boom - stick;
                // to get the twist, remove the wrist part; the remaining rotation around x is
                // twist.
                Rotation3d wrist3d = new Rotation3d(0, swingRelative3d.getY(), 0);
                Rotation3d twist3d = swingRelative3d.rotateBy(wrist3d.inverse());
                twist = OptionalDouble.of(twist3d.getX());
            }
        } else {
            // the end pose is not on the swing axis, so we can use the pose translation to
            // get the swing angle.
            Rotation2d swingAngle = translation.getAngle();
            swing = OptionalDouble.of(swingAngle.getRadians());
            // to get the wrist rotation, project the swing-relative rotation
            Rotation3d swing3d = new Rotation3d(swingAngle);
            Rotation3d swingRelative3d = endRotation.rotateBy(swing3d.inverse());
            // to get the stick-relative wrist angle, subtract the joints
            wrist = swingRelative3d.getY() - boom - stick;
            // to get the twist, remove the wrist part; the remaining rotation around x is
            // twist.
            Rotation3d wrist3d = new Rotation3d(0, swingRelative3d.getY(), 0);
            Rotation3d twist3d = swingRelative3d.rotateBy(wrist3d.inverse());
            // System.out.printf("twist3d %s\n", StrUtil.rotStr(twist3d));
            twist = OptionalDouble.of(twist3d.getX());

            if (MathUtil.isNear(-Math.PI / 2, endRotation.getY(), 0.01)) {
                // System.out.println("The wrist axis is straight up.");
            } else if (MathUtil.isNear(Math.PI / 2, endRotation.getY(), 0.01)) {
                // System.out.println("The wrist axis is straight down");
            } else {
                // Check that the swing angle matches the end angle.
                Rotation2d endRotation2d = endRotation.toRotation2d();
                if (Math.abs(endRotation2d.minus(swingAngle).getRadians()) > 1e-3) {
                    // throw new IllegalArgumentException(
                    // String.format("end rotation %s not equal to swing %s",
                    // endRotation2d, swingAngle));
                    // System.out.printf("end rotation %s not equal to swing %s for goal %s\n",
                    // endRotation2d, swingAngle, StrUtil.poseStr(end));
                }
            }
        }

        return new LynxArmConfig(
                swing.isPresent() ? OptionalDouble.of(MathUtil.angleModulus(swing.getAsDouble()))
                        : OptionalDouble.empty(),
                MathUtil.angleModulus(boom),
                MathUtil.angleModulus(stick),
                MathUtil.angleModulus(wrist),
                twist.isPresent() ? OptionalDouble.of(MathUtil.angleModulus(twist.getAsDouble()))
                        : OptionalDouble.empty());
    }

    /////////////////////////////////////////////////

    /** Translate along the x axis. */
    private static Transform3d x(double x) {
        return new Transform3d(x, 0, 0, new Rotation3d());
    }

    /** Translate along the z axis. */
    private static Transform3d z(double z) {
        return new Transform3d(0, 0, z, new Rotation3d());
    }

}
