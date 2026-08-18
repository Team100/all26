package org.team100.lib.subsystems.mecanum;

import org.team100.lib.dynamics.mecanum.MecanumDynamics;
import org.team100.lib.dynamics.mecanum.MecanumEffort;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.mecanum.MecanumKinematics100;
import org.team100.lib.kinematics.mecanum.MecanumKinematics100.Slip;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.VelocityControlSE2Logger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Mecanum drive with optional gyro.
 */
public class MecanumDrive100 extends SubsystemBase implements VelocitySubsystemSE2 {

    private final DoubleArrayLogger m_log_field_robot;
    private final VelocityControlSE2Logger m_log_input;
    /** May be null. */
    private final Gyro m_gyro;
    private final double m_trackWidthM;
    private final double m_wheelbaseM;
    private final LinearMechanism m_frontLeft;
    private final LinearMechanism m_frontRight;
    private final LinearMechanism m_rearLeft;
    private final LinearMechanism m_rearRight;
    private final MecanumKinematics100 m_kinematics;
    private final MecanumDynamics m_dynamics;

    private MecanumDriveWheelPositions m_positions;
    private VelocitySE2 m_input;
    private Pose2d m_pose;
    private Rotation2d m_gyroOffset;

    /**
     * Gyro may be null, in which case we use (not very accurate) odometry for yaw.
     */
    public MecanumDrive100(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            double m,
            double I,
            Gyro gyro,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            LinearMechanism frontLeft,
            LinearMechanism frontRight,
            LinearMechanism rearLeft,
            LinearMechanism rearRight) {
        LoggerFactory log = parent.type(this);
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_log_input = log.velocityControlSE2Logger(Level.TRACE, "drive input");
        m_gyro = gyro;
        m_trackWidthM = trackWidthM;
        m_wheelbaseM = wheelbaseM;
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        Translation2d fl = new Translation2d(m_wheelbaseM / 2, m_trackWidthM / 2);
        Translation2d fr = new Translation2d(m_wheelbaseM / 2, -m_trackWidthM / 2);
        Translation2d rl = new Translation2d(-m_wheelbaseM / 2, m_trackWidthM / 2);
        Translation2d rr = new Translation2d(-m_wheelbaseM / 2, -m_trackWidthM / 2);
        m_kinematics = new MecanumKinematics100(
                slip, fl, fr, rl, rr);
        m_dynamics = new MecanumDynamics(m, I, fl, fr, rl, rr);
        m_positions = new MecanumDriveWheelPositions();
        m_input = VelocitySE2.ZERO;
        m_pose = new Pose2d();
        m_gyroOffset = new Rotation2d();
    }

    @Override
    public ModelSE2 getState() {
        // assume the velocity is exactly what was requested.
        return new ModelSE2(m_pose, m_input);
    }

    /**
     * Use inverse kinematics to set wheel speeds.
     * 
     * @param nextV for the next timestep
     */
    @Override
    public void set(VelocityControlSE2 nextV) {
        Rotation2d yaw = getYaw();
        ChassisSpeeds speed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                nextV.velocity(), yaw);
        MecanumDriveWheelSpeeds mSpeed = m_kinematics.toWheelSpeeds(speed);
        ChassisAcceleration accel = ChassisAcceleration.fromFieldRelative(
                nextV.acceleration(), yaw);
        MecanumEffort effort = m_dynamics.effort(accel);
        m_frontLeft.setVelocity(mSpeed.frontLeftMetersPerSecond, effort.fl());
        m_frontRight.setVelocity(mSpeed.frontRightMetersPerSecond, effort.fr());
        m_rearLeft.setVelocity(mSpeed.rearLeftMetersPerSecond, effort.rl());
        m_rearRight.setVelocity(mSpeed.rearRightMetersPerSecond, effort.rr());
        m_log_input.log(() -> nextV);
    }

    private Rotation2d getYaw() {
        if (m_gyro == null)
            return m_pose.getRotation();
        return m_gyro.getYawNWU().minus(m_gyroOffset);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    /** Set the field-relative velocity. */
    public Command driveWithGlobalVelocity(VelocityControlSE2 v) {
        return run(() -> set(v))
                .withName("drive with global velocity");
    }

    public Command resetPose() {
        return runOnce(this::resetPoseAndGyro);
    }

    /** Set yaw to zero. */
    public Command resetYaw() {
        return runOnce(this::resetGyroOffset);
    }

    public void setPose(Pose2d p) {
        m_pose = p;
        if (m_gyro == null)
            return;
        m_gyroOffset = m_gyro.getYawNWU().minus(p.getRotation());
    }

    @Override
    public void periodic() {
        updatePose();
        m_log_field_robot.log(this::poseArray);
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    private void resetPoseAndGyro() {
        m_pose = new Pose2d();
        if (m_gyro == null)
            return;
        m_gyroOffset = m_gyro.getYawNWU();
    }

    private void resetGyroOffset() {
        if (m_gyro == null)
            return;
        m_gyroOffset = m_gyro.getYawNWU();
    }

    private void updatePose() {
        Twist2d twist = twist();
        m_pose = m_pose.exp(twist);
    }

    private Twist2d twist() {
        MecanumDriveWheelPositions newPositions = new MecanumDriveWheelPositions(
                m_frontLeft.getPositionM(),
                m_frontRight.getPositionM(),
                m_rearLeft.getPositionM(),
                m_rearRight.getPositionM());
        Twist2d twist = m_kinematics.toTwist2d(m_positions, newPositions);
        m_positions = newPositions;
        return twist;
    }

    private double[] poseArray() {
        return new double[] {
                m_pose.getX(),
                m_pose.getY(),
                m_pose.getRotation().getDegrees()
        };
    }

}
