package org.team100.lib.subsystems.swerve.kinodynamics;

import org.team100.lib.dynamics.swerve.SwerveDynamics;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.dynamics.swerve.Tire;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.subsystems.swerve.VeeringCorrection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Kinematics and dynamics of the swerve drive.
 * 
 * Includes speed limits, dynamic constraints, and kinematics.
 * 
 * This class represents *absolute maxima.*
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 * 
 * This used to include the steering rate, in order to provide
 * a suitable profile for profiled steering, but we don't do
 * profiled steering anymore, so it's gone.
 */
public class SwerveKinodynamics {
    // Geometry; these should be measured with a tape measure, not tuned.
    private final double m_fronttrack;
    private final double m_backtrack;
    private final double m_wheelbase;
    private final double m_frontoffset;
    private final Translation2d m_fl;
    private final Translation2d m_fr;
    private final Translation2d m_rl;
    private final Translation2d m_rr;
    private final double m_vcg;
    /** Diagonal distance from center to wheel. */
    private final double m_radius;
    /** Distance from the center to the nearest edge. */
    private final double m_fulcrum;
    private final SwerveDriveKinematics100 m_kinematics;
    private final SwerveDynamics m_dynamics;

    private final double m_maxDriveVelocityM_S;
    private final double m_stallAccelerationM_S2;
    private final double m_maxDriveAccelerationM_S2;
    private final double m_maxDriveDecelerationM_S2;

    /**
     * @param maxDriveVelocity        module drive speed m/s
     * @param stallAcceleration       acceleration at stall, used to compute
     *                                back-EMF-limited acceleration at higher RPMs,
     *                                resulting in an exponential velocity curve at
     *                                max output.
     * @param maxDriveAcceleration    module drive accel m/s^2, used for
     *                                constant-acceleration profiles. This should be
     *                                less than the stall acceleration, so that the
     *                                robot can stay ahead of the profile initially,
     *                                and fall behind as speed increases.
     * @param maxDriveDeceleration    module drive decel m/s^2. Should be higher
     *                                than accel limit, this is a positive number.
     * @param maxSteeringVelocity     module steering axis rate rad/s
     * @param maxSteeringAcceleration module steering axis accel rad/s^2
     * @param fronttrack              meters
     * @param backtrack               meters
     * @param wheelbase               meters
     * @param frontoffset             distance from the center of mass to the front
     *                                wheels, meters
     * @param vcg                     vertical center of gravity, meters
     * @param m                       mass, kg
     * @param I                       inertia, kg m^2
     * @param tire                    tire model for lateral force
     */
    SwerveKinodynamics(
            double maxDriveVelocity,
            double stallAcceleration,
            double maxDriveAcceleration,
            double maxDriveDeceleration,
            double fronttrack,
            double backtrack,
            double wheelbase,
            double frontoffset,
            double vcg,
            double m,
            double I,
            Tire tire) {

        // Measured quantities...
        m_fronttrack = fronttrack;
        m_backtrack = backtrack;
        m_wheelbase = wheelbase;
        m_frontoffset = frontoffset;
        m_vcg = vcg;
        m_fulcrum = Math.min(Math.min(m_fronttrack, m_backtrack) / 2, m_wheelbase / 2);
        m_radius = Math.hypot((fronttrack + backtrack) / 4, m_wheelbase / 2);
        m_fl = new Translation2d(m_frontoffset, m_fronttrack / 2);
        m_fr = new Translation2d(m_frontoffset, -m_fronttrack / 2);
        m_rl = new Translation2d(m_frontoffset - m_wheelbase, m_backtrack / 2);
        m_rr = new Translation2d(m_frontoffset - m_wheelbase, -m_backtrack / 2);
        m_kinematics = new SwerveDriveKinematics100(m_fl, m_fr, m_rl, m_rr);
        m_dynamics = new SwerveDynamics(m, I, tire, m_fl, m_fr, m_rl, m_rr);

        m_maxDriveVelocityM_S = maxDriveVelocity;
        m_stallAccelerationM_S2 = stallAcceleration;
        m_maxDriveAccelerationM_S2 = maxDriveAcceleration;
        m_maxDriveDecelerationM_S2 = maxDriveDeceleration;
    }

    /** Cruise speed, m/s. */
    public double getMaxDriveVelocityM_S() {
        return m_maxDriveVelocityM_S;
    }

    /**
     * Acceleration at stall, without current limiting. Used to compute
     * back-EMF-limited torque available at higher RPMs.
     */
    public double getStallAccelerationM_S2() {
        return m_stallAccelerationM_S2;
    }

    public double getMaxAngleStallAccelRad_S2() {
        return 12 * m_stallAccelerationM_S2 * m_radius
                / (m_fronttrack * m_backtrack + m_wheelbase * m_wheelbase);
    }

    /**
     * Motor-torque-limited acceleration rate, m/s^2. Used for constant-acceleration
     * profiles.
     */
    public double getMaxDriveAccelerationM_S2() {
        return m_maxDriveAccelerationM_S2;
    }

    /**
     * Motor-torque-limited drive deceleration rate, m/s^2. Motors are better at
     * slowing down than speeding up, so this should be larger than the accel rate.
     */
    public double getMaxDriveDecelerationM_S2() {
        return m_maxDriveDecelerationM_S2;
    }

    /** Spin cruise speed, rad/s. Computed from drive and frame size. */
    public double getMaxAngleSpeedRad_S() {
        return m_maxDriveVelocityM_S / m_radius;
    }

    /**
     * Motor-torque-limited spin accel rate, rad/s^2. Computed from drive and frame
     * size.
     */
    public double getMaxAngleAccelRad_S2() {
        return 12 * Math.max(
                m_maxDriveAccelerationM_S2,
                m_maxDriveDecelerationM_S2)
                * m_radius
                / (m_fronttrack * m_backtrack + m_wheelbase * m_wheelbase);
    }

    /**
     * Acceleration which will tip the robot onto two wheels, m/s^2. Computed from
     * vertical center of gravity and frame size.
     */
    public double getMaxCapsizeAccelM_S2() {
        return 9.8 * (m_fulcrum / m_vcg);
    }

    /**
     * Inverse kinematics, chassis speeds => module states.
     * 
     * The resulting state speeds are always positive.
     * 
     * This version does **DISCRETIZATION** to correct for swerve veering.
     * 
     * It also does extra veering correction proportional to rotation rate and
     * translational acceleration.
     * 
     * States may include empty angles for motionless wheels.
     * 
     * @param nextSpeed represents the desired speed for now+dt.
     */
    public SwerveModuleStates toSwerveModuleStates(ChassisSpeeds nextSpeed) {
        return toSwerveModuleStates(nextSpeed, TimedRobot100.LOOP_PERIOD_S);
    }

    /**
     * Effort to achieve the required acceleration, given the states. Includes both
     * longitudinal (provided by the motor) and lateral (provided by slip angle)
     * forces.
     */
    public SwerveEffort effort(SwerveModuleStates states, ChassisAcceleration a) {
        return m_dynamics.effort(states, a);
    }

    /**
     * Discretizes.
     * 
     * States may include empty angles for motionless wheels.
     * Otherwise angle is always within [-pi, pi].
     * 
     * @param nextSpeed represents the desired speed for now+dt.
     */
    SwerveModuleStates toSwerveModuleStates(ChassisSpeeds nextSpeed, double dt) {
        // This is the extra correction angle ...
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(nextSpeed.omegaRadiansPerSecond));
        // ... which is subtracted here; this isn't really a field-relative
        // transformation it's just a rotation.
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                nextSpeed.vxMetersPerSecond,
                nextSpeed.vyMetersPerSecond,
                nextSpeed.omegaRadiansPerSecond,
                angle);
        // discretization does not affect omega
        DiscreteSpeed descretized = discretize(chassisSpeeds, dt);
        SwerveModuleStates states = m_kinematics.inverse(descretized);
        return states;
    }

    /**
     * Given a desired instantaneous speed, extrapolate ahead one step, and return
     * the twist required to achieve that state.
     */
    public static DiscreteSpeed discretize(ChassisSpeeds chassisSpeeds, double dt) {
        Pose2d desiredDeltaPose = new Pose2d(
                chassisSpeeds.vxMetersPerSecond * dt,
                chassisSpeeds.vyMetersPerSecond * dt,
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * dt));

        return new DiscreteSpeed(Pose2d.kZero.log(desiredDeltaPose), dt);
    }

    /**
     * Returns the "instantaneous" chassis speeds corresponding to the module
     * states, i.e. the chassis speed pointing at the result of applying the module
     * state twist.
     * 
     * This could be used with odometry, but because odometry uses module positions
     * instead of velocities, it is not needed.
     * 
     * It performs inverse discretization and an extra correction.
     */
    public ChassisSpeeds toChassisSpeedsWithDiscretization(
            SwerveModuleStates moduleStates,
            double dt) {
        DiscreteSpeed discreteSpeeds = m_kinematics.forward(moduleStates, dt);
        Twist2d twist = discreteSpeeds.twist();

        Pose2d deltaPose = GeometryUtil.sexp(twist);
        ChassisSpeeds continuousSpeeds = new ChassisSpeeds(
                deltaPose.getX(),
                deltaPose.getY(),
                deltaPose.getRotation().getRadians()).div(dt);

        double omega = twist.dtheta / dt;
        // This is the opposite direction
        Rotation2d angle = new Rotation2d(VeeringCorrection.correctionRad(omega));
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                continuousSpeeds.vxMetersPerSecond,
                continuousSpeeds.vyMetersPerSecond,
                continuousSpeeds.omegaRadiansPerSecond,
                angle.unaryMinus());
    }

    /**
     * Robot-relative speed, without discretization.
     * This simply rotates the velocity from the field frame to the robot frame.
     */
    public static ChassisSpeeds toInstantaneousChassisSpeeds(
            VelocitySE2 v,
            Rotation2d theta) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                v.x(),
                v.y(),
                v.theta(),
                theta);
    }

    /**
     * Field-relative speed, without discretization.
     * This simply rotates the velocity from the robot frame to the field frame.
     */
    public static VelocitySE2 fromInstantaneousChassisSpeeds(ChassisSpeeds instantaneous, Rotation2d theta) {
        ChassisSpeeds c = ChassisSpeeds.fromRobotRelativeSpeeds(instantaneous, theta);
        return new VelocitySE2(c.vxMetersPerSecond, c.vyMetersPerSecond, c.omegaRadiansPerSecond);
    }

    public SwerveDriveKinematics100 getKinematics() {
        return m_kinematics;
    }

}