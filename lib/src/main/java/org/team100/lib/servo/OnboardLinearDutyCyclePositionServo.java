package org.team100.lib.servo;

import org.team100.lib.controller.r1.FeedbackR1;
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
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

import edu.wpi.first.math.MathUtil;

/**
 * Position control using duty cycle feature of linear mechanism
 */
public class OnboardLinearDutyCyclePositionServo implements LinearPositionServo {
    private static final double POSITION_TOLERANCE = 0.01;
    private static final double VELOCITY_TOLERANCE = 0.01;
    private final LinearMechanism m_mechanism;
    private final PDynamics m_dynamics;
    private final ReferenceR1 m_ref;
    private final FeedbackR1 m_feedback;
    private final double m_kV;

    /**
     * This is an awful hack that accounts for the real kT and also
     * the gearing and drive diameter.
     */
    private final double m_kT;

    private final DoubleLogger m_log_goal;
    private final SetpointsR1Logger m_log_setpoints;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_acceleration;
    private final DoubleLogger m_log_position_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_log_accel_error;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final BooleanLogger m_log_at_setpoint;
    private final BooleanLogger m_log_profile_done;
    private final BooleanLogger m_log_at_goal;

    /** Null if there's no current profile. */
    private StateR1 m_goal;
    private SetpointsR1 m_setpoints;

    public OnboardLinearDutyCyclePositionServo(
            LoggerFactory parent,
            LinearMechanism mechanism,
            PDynamics dynamics,
            ReferenceR1 ref,
            FeedbackR1 feedback,
            double kV,
            double kT) {
        LoggerFactory log = parent.type(this);
        m_mechanism = mechanism;
        m_dynamics = dynamics;
        m_ref = ref;
        m_feedback = feedback;
        m_kV = kV;
        m_kT = kT;
        m_log_goal = log.doubleLogger(Level.TRACE, "goal (m)");
        m_log_setpoints = log.setpointsR1Logger(Level.TRACE, "setpoints");
        m_log_position = log.doubleLogger(Level.TRACE, "position (m)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (m_s)");
        m_log_acceleration = log.doubleLogger(Level.TRACE, "accel (m_s2)");
        m_log_position_error = log.doubleLogger(Level.COMP, "position error (m)");
        m_log_velocity_error = log.doubleLogger(Level.COMP, "velocity error (m_s)");
        m_log_accel_error = log.doubleLogger(Level.COMP, "accel error (m_s2)");
        m_log_u_FB = log.doubleLogger(Level.TRACE, "u_FB (duty cycle)");
        m_log_u_FF = log.doubleLogger(Level.TRACE, "u_FF (duty cycle)");
        m_log_u_TOTAL = log.doubleLogger(Level.TRACE, "u_TOTAL (duty cycle)");
        m_log_at_setpoint = log.booleanLogger(Level.TRACE, "at setpoint");
        m_log_profile_done = log.booleanLogger(Level.TRACE, "profile done");
        m_log_at_goal = log.booleanLogger(Level.TRACE, "at goal");
        LogPoller.register(this::log);
    }

    @Override
    public void reset() {
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        // OptionalDouble velocity = getVelocity();
        // if (velocity.isEmpty())
        // return;
        ControlR1 measurement = new ControlR1(getPosition(), 0);
        m_setpoints = new SetpointsR1(measurement, measurement);
        StateR1 state = measurement.state();
        m_goal = state;
        m_ref.setGoal(state);
        // reference is initalized with measurement only here.
        m_ref.init(state);
        // m_controller.init(m_setpoint.model());
        m_feedback.reset();
    }

    @Override
    public void setVoltage(double v) {
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

    /** Resets the profile if necessary. */
    @Override
    public void setPositionProfiled(double goalM) {
        m_log_goal.log(() -> goalM);
        StateR1 goal = new StateR1(goalM, 0);

        if (!goal.near(m_goal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_setpoints.next().state());
        }
        actuate(m_ref.get());
    }

    /** Invalidates the current profile */
    @Override
    public void setPositionDirect(double goalM) {
        m_goal = null;
        ControlR1 c = new ControlR1(goalM);
        actuate(new SetpointsR1(c, c));
    }

    /**
     * Compute feedback using the current setpoint, feedforward using the next
     * setpoint, and actuate using duty cycle.
     * Ignores torque
     */
    private void actuate(SetpointsR1 setpoints) {
        // setpoint must be updated so the profile can see it
        m_setpoints = setpoints;
        double velocityM_S = m_setpoints.next().v();
        double accelM_S2 = m_setpoints.next().a();
        PEffort t = m_dynamics.effort(new PAcceleration(accelM_S2));

        final double position = getPosition();
        final double velocity = getVelocity();
        final StateR1 measurement = new StateR1(position, velocity);

        final double u_FF = m_kV * velocityM_S + m_kT * t.f();
        final double u_FB = m_feedback.calculate(measurement, setpoints.current().state());
        final double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);

        m_mechanism.setDutyCycle(u_TOTAL);

        m_log_setpoints.log(() -> setpoints);

        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);

        m_log_position_error.log(() -> setpoints.current().x() - position);
        m_log_velocity_error.log(() -> setpoints.current().v() - velocity);
        m_log_accel_error.log(() -> setpoints.current().a() - getAcceleration());
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
        return Math.abs(pErr) < POSITION_TOLERANCE
                && Math.abs(vErr) < VELOCITY_TOLERANCE;
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
