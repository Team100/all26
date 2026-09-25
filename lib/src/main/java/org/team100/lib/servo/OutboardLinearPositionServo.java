package org.team100.lib.servo;

import org.team100.lib.dynamics.p.PAcceleration;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.dynamics.p.PEffort;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.SetpointsR1Logger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

/**
 * Profiled or direct position control using the feedback controller in the
 * motor controller hardware.
 */
public class OutboardLinearPositionServo implements LinearPositionServo {
    private static final boolean DEBUG = false;
    private final LinearMechanism m_mechanism;
    private final PDynamics m_dynamics;
    private final ReferenceR1 m_ref;
    private final double m_positionTolerance;
    private final double m_velocityTolerance;

    private final DoubleLogger m_log_goal;
    private final SetpointsR1Logger m_log_setpoints;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_acceleration;
    private final DoubleLogger m_log_position_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_log_accel_error;
    private final DoubleLogger m_log_ff_torque;
    private final BooleanLogger m_log_at_setpoint;
    private final BooleanLogger m_log_profile_done;
    private final BooleanLogger m_log_at_goal;

    /** Null if there's no current profile. */
    private StateR1 m_goal;
    private SetpointsR1 m_setpoints;

    public OutboardLinearPositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            PDynamics dynamics,
            ReferenceR1 ref,
            double positionTolerance,
            double velocityTolerance) {
        LoggerFactory log = parent.type(this);
        m_mechanism = mechanism;
        m_dynamics = dynamics;
        m_ref = ref;
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
        m_log_goal = log.doubleLogger(Level.COMP, "goal (m)");
        m_log_setpoints = log.setpointsR1Logger(Level.COMP, "setpoints");
        m_log_position = log.doubleLogger(Level.COMP, "position (m)");
        m_log_velocity = log.doubleLogger(Level.COMP, "velocity (m_s)");
        m_log_acceleration = log.doubleLogger(Level.COMP, "accel (m_s2)");
        m_log_position_error = log.doubleLogger(Level.COMP, "position error (m)");
        m_log_velocity_error = log.doubleLogger(Level.COMP, "velocity error (m_s)");
        m_log_accel_error = log.doubleLogger(Level.COMP, "accel error (m_s2)");
        m_log_ff_torque = log.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_at_setpoint = log.booleanLogger(Level.TRACE, "at setpoint");
        m_log_profile_done = log.booleanLogger(Level.TRACE, "profile done");
        m_log_at_goal = log.booleanLogger(Level.TRACE, "at goal");
        LogPoller.register(this::log);
    }

    /**
     * Make a servo from a motor and a position reference.
     * Creates the mechanism in between, without soft limits.
     */
    public static OutboardLinearPositionServo make(
            LoggerFactory log,
            Motor motor,
            PDynamics dyn,
            ReferenceR1 ref,
            double gearRatio,
            double wheelDiameterM) {
        LinearMechanism mech = new LinearMechanism(
                log, motor, motor.encoder(), gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearPositionServo(
                log, mech, dyn, ref, 0.01, 0.01);
    }

    @Override
    public void reset() {
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        ControlR1 measurement = new ControlR1(getPosition(), 0);
        m_setpoints = new SetpointsR1(measurement, measurement);
        // reference is initalized with measurement only here.
        StateR1 state = measurement.state();
        if (DEBUG)
            System.out.printf("reset goal to measurement %s\n", state);
        m_goal = state;
        m_ref.setGoal(state);
        m_ref.init(state);
    }

    @Override
    public void setVoltage(double v) {
        m_goal = null;
        m_setpoints = null;
        m_mechanism.setVoltage(v);
    }

    @Override
    public void setEncoderPositionM(double positionM) {
        m_mechanism.setEncoderPositionM(positionM);
    }

    @Override
    public void setVelocity(double velocityM_S) {
        m_mechanism.setVelocity(velocityM_S, 0);
    }

    /** Resets the profile if necessary */
    @Override
    public void setPositionProfiled(double goalM) {
        m_log_goal.log(() -> goalM);
        StateR1 goal = new StateR1(goalM, 0);

        if (!goal.near(m_goal, m_positionTolerance, m_velocityTolerance)) {
            if (DEBUG)
                System.out.printf("adjust goal old %s new %s\n", m_goal, goal);
            m_goal = goal;
            m_ref.setGoal(goal);
            if (m_setpoints == null) {
                // erased by dutycycle control
                ControlR1 c = new ControlR1(getPosition(), 0);
                m_setpoints = new SetpointsR1(c, c);
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_setpoints.next().state());
        } else {
            if (DEBUG)
                System.out.printf("goals are near: old %s new %s\n", m_goal, goal);
        }
        actuate(m_ref.get());
    }

    /**
     * Passthrough to the outboard control.
     * Invalidates the current profile.
     * Uses the same setpoint for "current" and "next".
     */
    @Override
    public void setPositionDirect(double goalM) {
        m_goal = null;
        ControlR1 c = new ControlR1(goalM);
        actuate(new SetpointsR1(c, c));
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * For outboard control we only use the "next" setpoint.
     * 
     * Gravity compensation used to be here; it should be in the
     * dynamics now.
     */
    private void actuate(SetpointsR1 setpoints) {
        // setpoint must be updated so the profile can see it
        m_setpoints = setpoints;
        double positionM = m_setpoints.next().x();
        double velocityM_S = m_setpoints.next().v();
        double accelM_S2 = m_setpoints.next().a();
        PEffort t = m_dynamics.effort(new PAcceleration(accelM_S2));
        m_mechanism.setPosition(
                positionM,
                velocityM_S,
                t.f());
        m_log_setpoints.log(() -> setpoints);
        m_log_ff_torque.log(() -> t.f());
        m_log_position_error.log(() -> setpoints.current().x() - getPosition());
        m_log_velocity_error.log(() -> setpoints.current().v() - getVelocity());
        m_log_accel_error.log(() -> setpoints.current().a() - getAcceleration());
    }

    /** This is for wrist feedforard. */
    public double getSetpointAcceleration() {
        return m_setpoints.next().a();
    }

    /** Invalidates the current profile */
    public void setDutyCycle(double value) {
        m_goal = null;
        m_setpoints = null;
        m_mechanism.setDutyCycle(value);
    }

    @Override
    public double getPosition() {
        return m_mechanism.getPositionM();
    }

    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public double getAcceleration() {
        return m_mechanism.getAccelerationM_S2();
    }

    @Override
    public boolean atSetpoint() {
        if (m_setpoints == null)
            return false;
        // compare current setpoint to measurement
        double pErr = m_setpoints.current().x() - m_mechanism.getPositionM();
        double vErr = m_setpoints.current().v() - m_mechanism.getVelocityM_S();
        return Math.abs(pErr) < m_positionTolerance
                && Math.abs(vErr) < m_velocityTolerance;
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
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    @Override
    public void stop() {
        m_goal = null;
        m_setpoints = null;
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    private void log() {
        m_log_position.log(() -> getPosition());
        m_log_velocity.log(() -> getVelocity());
        m_log_acceleration.log(() -> getAcceleration());
        m_log_at_setpoint.log(() -> atSetpoint());
        m_log_profile_done.log(() -> profileDone());
        m_log_at_goal.log(() -> atGoal());
    }
}
