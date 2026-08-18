package org.team100.lib.subsystems.lynxmotion_arm;

import java.util.function.DoubleSupplier;

import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.kinematics.lynx_arm.LynxArmKinematics;
import org.team100.lib.motor.servo.CalibratedServo;
import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveCommand;
import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveGrip;
import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveManually;
import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveXY;
import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveZ;
import org.team100.lib.subsystems.lynxmotion_arm.commands.ToggleGrip;
import org.team100.lib.subsystems.lynxmotion_arm.commands.ToggleHeight;
import org.team100.lib.util.AffineFunction;
import org.team100.lib.util.Clamp;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Lynxmotion AL5D trainer board.
 * 
 * The arm is a serial mechanism, and so it implements a series of
 * three-dimensional transforms: rotations (the joints) and translations (along
 * the links).
 * 
 * Since the AL5D is mostly planar, we're using the 2d convention, where each
 * link is a translation along its own x, and each joint rotation is also
 * aligned with the same parent link x.
 * 
 * This means that the "all joints at zero" state for the arm should be
 * stretched out along the world x axis.
 * 
 * overview:
 * 
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 *
 * calibration:
 * 
 * https://docs.google.com/spreadsheets/d/1XCtQGnJABVWTuCkx0t6u_xJjezelqyBE2Rz7eqmoyh4
 * 
 * background:
 * 
 * https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html
 */
public class LynxArm extends SubsystemBase implements AutoCloseable {
    private static final boolean DEBUG = false;
    // prefer vertical grip. beyond this radius, extend it.
    private static final double VERTICAL_LIMIT = 0.3;
    private static final double MAX_RADIUS = 0.4;
    private static final Pose3d HOME = new Pose3d(0.2, 0, 0.2, new Rotation3d(0, Math.PI / 4, 0));

    private final CalibratedServo m_swing;
    private final CalibratedServo m_boom;
    private final CalibratedServo m_stick;
    private final CalibratedServo m_wrist;
    private final CalibratedServo m_twist;
    // it might be easier if the grip were actually a separate subsystem,
    // since it never collides with anything...
    private final CalibratedServo m_grip;

    private final LynxArmKinematics m_kinematics;

    private boolean distanceMode;
    private final double MODE_BARRIER = 2;

    public LynxArm(LynxArmKinematics kinematics) {
        m_kinematics = kinematics;
        // all these implement the WPI normal coordinates:
        // x ahead, y left, z up.

        // yaw; joint zero is is in the middle of the servo range; unconstrained.
        m_swing = new CalibratedServo(0,
                new Clamp(-Math.PI / 2, Math.PI / 2),
                new AffineFunction(-3.216, 1.534));

        // pitch; joint zero and servo zero are aligned; unconstrained.
        m_boom = new CalibratedServo(1,
                new Clamp(-Math.PI, 0),
                new AffineFunction(-3.437, 0.138));

        // pitch; joint zero is servo max; constrained in the positive direction.
        m_stick = new CalibratedServo(2,
                new Clamp(0, Math.PI),
                new AffineFunction(3.205, -0.125));

        // pitch; joint zero is in the middle of the servo range; unconstrained
        m_wrist = new CalibratedServo(3,
                new Clamp(-Math.PI / 2, Math.PI / 2),
                new AffineFunction(-3.203, 1.568));

        // roll: note the limited range.
        m_twist = new CalibratedServo(4,
                new Clamp(-1.257, 1.187),
                new AffineFunction(-3.081, 1.54));

        // the grip axis measures the width of the jaws.
        m_grip = new CalibratedServo(5,
                new Clamp(0, 0.033),
                new AffineFunction(-0.041, 0.036));

        // initialize the servo values so that the solver doesn't freak out
        m_swing.set(0);
        m_boom.set(-2.0 * Math.PI / 3);
        m_stick.set(Math.PI / 2);
        m_wrist.set(Math.PI / 2);
        m_twist.set(0);
        m_grip.set(0.02);

        // set the initial position to the actual desired value
        if (DEBUG) {
            System.out.println("-> initializing position to HOME");
            System.out.printf("HOME: %s\n", StrUtil.poseStr(HOME));
        }
        setPosition(HOME);
        if (DEBUG)
            System.out.println("-> check position");
        getPosition();
        if (DEBUG)
            System.out.println("-> initializing done");
    }



    /**
     * Sets the end-effector pose.
     * 
     * Moves in an uncoordinated way, so the new position should be close to the
     * current position. Coordination should be handled by the caller.
     * 
     * For indeterminate axes, we do nothing.
     * 
     * Pose does not include grip state.
     */
    public void setPosition(Pose3d end) {
        if (DEBUG)
            System.out.println("setPosition()");
        LynxArmConfig q = getInverse(end);
        if (DEBUG)
            System.out.printf("set q: %s\n", q.str());

        q.swing().ifPresent(m_swing::set);
        m_boom.set(q.boom());
        m_stick.set(q.stick());
        m_wrist.set(q.wrist());
        q.twist().ifPresent(m_twist::set);
        if (q.swing().isEmpty())
            if (DEBUG)
                System.out.println("empty swing");
        if (q.twist().isEmpty())
            if (DEBUG)
                System.out.println("empty twist");
    }

    /**
     * Compute the pose that gets to this translation, using the current z value.
     * There's no feasibility check here.
     */
    public void setPosition(Translation2d t) {
        Pose3d p = getPosition().p6();
        final double pitch;
        double r = t.getNorm();
        if (r < VERTICAL_LIMIT) {
            // for near targets, the grip is vertical.
            pitch = Math.PI / 2;
        } else {
            // further away, the pitch depends on range.
            double s = 1 - (r - VERTICAL_LIMIT) / (MAX_RADIUS - VERTICAL_LIMIT);
            pitch = s * Math.PI / 2;
        }

        Pose3d newPose = new Pose3d(
                new Translation3d(t.getX(), t.getY(), p.getZ()),
                new Rotation3d(0.0, pitch, t.getAngle().getRadians()));
        setPosition(newPose);
    }

    /** change z, leave everything else alone. */
    public void setHeight(double z) {
        Pose3d p = getPosition().p6();
        setPosition(new Pose3d(
                new Translation3d(p.getX(), p.getY(), z),
                p.getRotation()));
    }

    public void setGrip(double width) {
        m_grip.set(width);
    }

    public double getGrip() {
        return m_grip.get();
    }

    public LynxArmPose getPosition() {
        if (DEBUG)
            System.out.println("getPosition()");
        LynxArmConfig q = getMeasuredConfig();
        LynxArmPose p = m_kinematics.forward(q);
        if (DEBUG)
            System.out.printf("p6: %s\n", StrUtil.poseStr(p.p6()));
        return p;
    }

    public void changeMode() {
        Pose3d currentPose = this.getPosition().p6();
        
        double x = currentPose.getX();
        double y = currentPose.getY();
        double r = Math.hypot(x, y);

        if (r < MODE_BARRIER)
            distanceMode = true;
        else
            distanceMode = false;
    }

    public boolean getDistanceMode(){
        return distanceMode;
    }

    public Command changeModeCmd() {
        return runOnce(this::changeMode);
    }

    public Command up() {
        return new MoveZ(this, 0.05);
    }

    public Command down() {
        return new MoveZ(this, 0.0);
    }

    public Command closeGrip() {
        return new MoveGrip(this, 0);
    }

    public Command openGrip() {
        return new MoveGrip(this, 0.02);
    }

    /** Move to goal forever -- use a termination condition to stop. */
    public MoveCommand moveTo(Pose3d goal) {
        return new MoveCommand(this, goal, 0.1);
    }

    /** Move to goal and terminate when done. */
    public Command moveQuicklyUntilDone(Pose3d goal) {
        MoveCommand m = new MoveCommand(this, goal, 0.5);
        return m.until(m::done);
    }

    public Command moveXY(double x, double y) {
        MoveXY m = new MoveXY(this, new Translation2d(x, y));
        return m.until(m::done);
    }

    public MoveCommand moveHome() {
        return new MoveCommand(this, HOME, 0.1);
    }

    public MoveManually manual(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier zSpeed) {
        return new MoveManually(this, xSpeed, ySpeed, zSpeed);
    }

    public ToggleGrip toggleGrip() {
        return new ToggleGrip(this);
    }

    public ToggleHeight toggleHeight() {
        return new ToggleHeight(this);
    }

    @Override
    public void close() {
        m_swing.close();
        m_boom.close();
        m_stick.close();
        m_wrist.close();
        m_twist.close();
        m_grip.close();
    }

    /////////////////////////////////////////////

    public LynxArmConfig getInverse(Pose3d p) {
        if (DEBUG)
            System.out.println("getInverse()");
        LynxArmConfig q0 = getMeasuredConfig();
        // p might be infeasible, so fix it.
        p = LynxArmKinematics.fix(p);
        return m_kinematics.inverse(q0, p);
    }

    public LynxArmConfig getMeasuredConfig() {
        if (DEBUG)
            System.out.println("getMeasuredConfig()");
        LynxArmConfig q = new LynxArmConfig(
                m_swing.get(),
                m_boom.get(),
                m_stick.get(),
                m_wrist.get(),
                m_twist.get());
        if (DEBUG)
            System.out.printf("get q: %s\n", q.str());
        return q;
    }
}
