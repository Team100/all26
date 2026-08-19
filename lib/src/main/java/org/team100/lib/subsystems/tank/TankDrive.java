package org.team100.lib.subsystems.tank;

import org.team100.lib.dynamics.differential.DifferentialDriveDynamics;
import org.team100.lib.dynamics.differential.DifferentialDriveEffort;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisVelocitiesLogger;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.visualization.VizUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.drive.DifferentialDrive;
import org.wpilib.drive.DifferentialDrive.WheelVelocities;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.DifferentialDriveKinematics;
import org.wpilib.math.kinematics.DifferentialDriveWheelPositions;
import org.wpilib.math.kinematics.DifferentialDriveWheelVelocities;

/**
 * Tank drive that uses two linear mechanisms and provides a pose estimate using
 * odometry only.
 */
public class TankDrive extends SubsystemBase {
    private final DoubleArrayLogger m_log_field_robot;
    private final DifferentialDriveDynamics m_dynamics;
    private final double m_trackWidthM;
    private final double m_maxSpeedM_S;
    private final LinearMechanism m_left;
    private final LinearMechanism m_right;
    private final DifferentialDriveKinematics m_kinematics;

    private final ChassisVelocitiesLogger m_logChassisVelocities;
    private final DoubleLogger m_logLeft;
    private final DoubleLogger m_logRight;

    private DifferentialDriveWheelPositions m_positions;
    private Pose2d m_pose;

    public TankDrive(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            DifferentialDriveDynamics dynamics,
            double trackWidthM,
            double maxSpeedM_S,
            LinearMechanism left,
            LinearMechanism right) {
        LoggerFactory log = parent.type(this);
        m_dynamics = dynamics;
        m_logChassisVelocities = log.ChassisVelocitiesLogger(Level.TRACE, "chassis speeds");
        m_logLeft = log.doubleLogger(Level.TRACE, "left");
        m_logRight = log.doubleLogger(Level.TRACE, "right");
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_trackWidthM = trackWidthM;
        m_maxSpeedM_S = maxSpeedM_S;
        m_left = left;
        m_right = right;
        m_kinematics = new DifferentialDriveKinematics(m_trackWidthM);
        m_positions = new DifferentialDriveWheelPositions(0, 0);
        m_pose = new Pose2d();
    }

    /** Use arcade drive to set duty cycle directly. */
    public void setDutyCycle(double translationSpeed, double rotSpeed) {
        WheelVelocities s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setDutyCycle(s.left);
        m_right.setDutyCycle(s.right);
    }

    /**
     * Use inverse kinematics to set wheel speeds.
     * 
     * New! Uses dynamics to compute motor forces.
     * 
     * Ignores lateral acceleration.
     */
    public void setVelocity(ChassisVelocities speed, ChassisAcceleration accel) {
        DifferentialDriveWheelVelocities wheelSpeeds = m_kinematics.toWheelVelocities(speed);
        double left = wheelSpeeds.left;
        double right = wheelSpeeds.right;

        DifferentialDriveEffort effort = m_dynamics.effort(accel);
        m_left.setVelocity(left, effort.F1());
        m_right.setVelocity(right, effort.F2());

        m_logChassisVelocities.log(() -> speed);
        m_logLeft.log(() -> left);
        m_logRight.log(() -> right);
    }

    /** For manual driving, to derive a feasible setpoint */
    public ChassisVelocities desaturate(double translationM_S, double rotationRad_S) {
        ChassisVelocities speed = new ChassisVelocities(translationM_S, 0, rotationRad_S);
        DifferentialDriveWheelVelocities ws = m_kinematics.toWheelVelocities(speed);
        ws = ws.desaturate(m_maxSpeedM_S);
        ChassisVelocities actual = m_kinematics.toChassisVelocities(ws);
        return actual;
    }

    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    @Override
    public void periodic() {
        updatePose();
        m_log_field_robot.log(this::poseArray);
        m_left.periodic();
        m_right.periodic();
    }

    public void setPose(Pose2d p) {
        m_pose = p;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    /** Set the drive velocity to a constant, for very simple auto. */
    public Command driveWithVelocity(
            double velM_S, double omegaRad_S,
            double accelM_S2, double alphaRad_S2) {
        return run(() -> {
            ChassisVelocities speed = new ChassisVelocities(velM_S, 0, omegaRad_S);
            ChassisAcceleration accel = new ChassisAcceleration(accelM_S2, 0, alphaRad_S2);
            setVelocity(speed, accel);
        }).withName("drive with velocity");
    }

    private void updatePose() {
        // This twist is relative to the center of rotation, which is near the midpoint
        // of the drive wheel axis, not the center of the robot, unless the drive wheels
        // happen to be in the center.
        Twist2d twist = twist();
        m_pose = m_pose.plus(twist.exp());
    }

    private Twist2d twist() {
        DifferentialDriveWheelPositions newPositions = new DifferentialDriveWheelPositions(
                m_left.getPositionM(),
                m_right.getPositionM());
        Twist2d twist = m_kinematics.toTwist2d(m_positions, newPositions);
        m_positions = newPositions;
        return twist;
    }

    private double[] poseArray() {
        return VizUtil.poseToArray(m_pose);
    }
}
