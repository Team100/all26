package org.team100.lib.servo;

import org.team100.lib.dynamics.p.PAcceleration;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.dynamics.p.PEffort;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.VelocitySetpointsR1Logger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.reference.r1.VelocitySetpointsR1;
import org.team100.lib.state.VelocityControlR1;

import edu.wpi.first.math.MathUtil;

/**
 * Profiled or direct velocity control using the feedback controller in the
 * motor controller hardware.
 * 
 * WARNING: the velocity control in REV motors is not very good. If you must use
 * it, you'll want to reduce the filtering on the sensor.
 */
public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;

    private final LinearMechanism m_mechanism;
    private final PDynamics m_dynamics;
    private final VelocityReferenceR1 m_ref;
    private final double m_toleranceM_S;

    private final DoubleLogger m_log_goal;
    private final VelocitySetpointsR1Logger m_log_setpoints;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_acceleration;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_log_accel_error;
    private final BooleanLogger m_log_at_setpoint;
    private final BooleanLogger m_log_profile_done;
    private final BooleanLogger m_log_at_goal;

    // null if no profile
    private Double m_goal;
    private VelocitySetpointsR1 m_setpoints;

    public OutboardLinearVelocityServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            PDynamics dynamics,
            VelocityReferenceR1 ref,
            double toleranceM_S) {
        LoggerFactory log = parent.type(this);
        m_mechanism = mechanism;
        m_dynamics = dynamics;
        m_ref = ref;
        m_toleranceM_S = toleranceM_S;
        m_log_goal = log.doubleLogger(Level.COMP, "goal (m_s)");
        m_log_setpoints = log.velocitySetpointsR1Logger(Level.COMP, "setpoints");
        m_log_velocity = log.doubleLogger(Level.COMP, "velocity (m_s)");
        m_log_acceleration = log.doubleLogger(Level.COMP, "accel (m_s2)");
        m_log_velocity_error = log.doubleLogger(Level.COMP, "velocity error (m_s)");
        m_log_accel_error = log.doubleLogger(Level.COMP, "accel error (m_s2)");
        m_log_at_setpoint = log.booleanLogger(Level.TRACE, "at setpoint");
        m_log_profile_done = log.booleanLogger(Level.TRACE, "profile done");
        m_log_at_goal = log.booleanLogger(Level.TRACE, "at goal");
        LogPoller.register(this::log);
    }

    /**
     * Make a servo from a motor and a velocity reference.
     * Creates the mechanism in between.
     */
    public static OutboardLinearVelocityServo make(
            LoggerFactory log,
            Motor motor,
            PDynamics dynamics,
            VelocityReferenceR1 ref,
            double gearRatio,
            double wheelDiameterM,
            double toleranceM_S) {
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(
                log, mech, dynamics, ref, toleranceM_S);
    }

    @Override
    public void reset() {
        VelocityControlR1 measurement = new VelocityControlR1(getVelocity(), 0);
        m_setpoints = new VelocitySetpointsR1(measurement, measurement);
        double v = measurement.v();
        if (DEBUG)
            System.out.printf("reset goal to measurement %6.3f\n", v);
        m_goal = v;
        m_ref.setGoal(v);
        m_ref.init(v);
    }

    @Override
    public void setVoltage(double v) {
        m_mechanism.setVoltage(v);
    }

    /**
     * Resets the profile if necessary.
     * 
     * Must be called periodically to progress through the profile.
     * 
     * Will not work in Command.initialize().
     */
    @Override
    public void setVelocityProfiled(double goalM_S) {
        m_log_goal.log(() -> goalM_S);
        if (m_goal == null || !MathUtil.isNear(goalM_S, m_goal, m_toleranceM_S)
                || !m_ref.valid()) {
            m_goal = goalM_S;
            m_ref.setGoal(goalM_S);
            if (m_setpoints == null) {
                VelocityControlR1 c = new VelocityControlR1(getVelocity(), 0);
                m_setpoints = new VelocitySetpointsR1(c, c);
            }
            m_ref.init(m_setpoints.next().v());
        } else {
            if (DEBUG)
                System.out.printf("goals are near: old %s new %s\n", m_goal, goalM_S);
        }
        actuate(m_ref.get());
    }

    /** Invalidates the current profile */
    @Override
    public void setDutyCycle(double dutyCycle) {
        m_goal = null;
        m_setpoints = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     * Uses the same setpoint for "current" and "next".
     */
    @Override
    public void setVelocityDirect(double setpointM_S) {
        m_goal = null;
        VelocityControlR1 c = new VelocityControlR1(setpointM_S, 0);
        actuate(new VelocitySetpointsR1(c, c));
    }

    private void actuate(VelocitySetpointsR1 setpoints) {
        m_setpoints = setpoints;
        double velocityM_S = m_setpoints.next().v();
        double accelM_S2 = m_setpoints.next().a();
        PEffort t = m_dynamics.effort(new PAcceleration(accelM_S2));
        m_mechanism.setVelocity(velocityM_S, t.f());
        m_log_setpoints.log(() -> m_setpoints);
        m_log_velocity_error.log(() -> setpoints.current().v() - getVelocity());
        m_log_accel_error.log(() -> setpoints.current().a() - getAcceleration());
    }

    /**
     * Current velocity measurement. Note this can be noisy, maybe filter it.
     */
    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public double getAcceleration() {
        return m_mechanism.getAccelerationM_S2();
    }

    /**
     * Velocity error (m/s).
     * Positive error = too slow, negative error = too fast.
     */
    public double error() {
        if (m_goal == null) {
            return 0;
        }
        return m_goal - m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    /** invalidates the current goal and setpoint */
    @Override
    public void stop() {
        m_goal = null;
        m_setpoints = null;
        m_mechanism.stop();
    }

    @Override
    public double getDistance() {
        return m_mechanism.getPositionM();
    }

    @Override
    public void play(double freq) {
        m_mechanism.play(freq);
    }

    @Override
    public boolean atSetpoint() {
        if (m_setpoints == null)
            return false;
        double vErr = m_setpoints.current().v() - m_mechanism.getVelocityM_S();
        return Math.abs(vErr) < m_toleranceM_S;
    }

    @Override
    public boolean profileDone() {
        if (m_goal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    private void log() {
        m_log_velocity.log(() -> getVelocity());
        m_log_acceleration.log(() -> getAcceleration());
        m_log_at_setpoint.log(() -> atSetpoint());
        m_log_profile_done.log(() -> profileDone());
        m_log_at_goal.log(() -> atGoal());
    }
}
