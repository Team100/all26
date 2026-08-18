package org.team100.lib.servo;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.dynamics.p.PAcceleration;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.dynamics.p.PEffort;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

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
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final ControlR1Logger m_log_setpoint;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    /** Null if there's no current profile. */
    private ModelR1 m_goal;
    private ControlR1 m_setpoint;

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
        m_log_position = log.doubleLogger(Level.TRACE, "position (m)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (m_s)");
        m_log_setpoint = log.ControlR1Logger(Level.TRACE, "setpoint (m)");
        m_log_u_FB = log.doubleLogger(Level.TRACE, "u_FB (duty cycle)");
        m_log_u_FF = log.doubleLogger(Level.TRACE, "u_FF (duty cycle)");
        m_log_u_TOTAL = log.doubleLogger(Level.TRACE, "u_TOTAL (duty cycle)");
        m_log_error = log.doubleLogger(Level.TRACE, "Controller Position Error (m)");
        m_log_velocity_error = log.doubleLogger(Level.TRACE, "Controller Velocity Error (m_s)");
    }

    @Override
    public void reset() {
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        // OptionalDouble velocity = getVelocity();
        // if (velocity.isEmpty())
        // return;
        ControlR1 measurement = new ControlR1(getPosition(), 0);
        m_setpoint = measurement;
        m_ref.setGoal(measurement.model());
        // reference is initalized with measurement only here.
        m_ref.init(measurement.model());
        // m_controller.init(m_setpoint.model());
        m_feedback.reset();
    }

    /**
     * Resets the profile if necessary.
     * 
     * @param goalM
     * @param feedForwardTorqueNm ignored
     */
    @Override
    public void setPositionProfiled(double goalM) {
        m_log_goal.log(() -> goalM);
        final ModelR1 goal = new ModelR1(goalM, 0);

        if (!goal.near(m_goal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            m_goal = goal;
            m_ref.setGoal(goal);
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_setpoint.model());
        }
        actuate(m_ref.get());
    }

    /**
     * Invalidates the current profile
     * 
     * @param setpoints
     * @param feedForwardTorqueNm ignored
     */
    @Override
    public void setPositionDirect(SetpointsR1 setpoints) {
        m_goal = null;
        actuate(setpoints);
    }

    /**
     * Compute feedback using the current setpoint, feedforward using the next
     * setpoint, and actuate using duty cycle.
     * Ignores torque
     */
    private void actuate(SetpointsR1 setpoints) {
        // setpoint must be updated so the profile can see it
        m_setpoint = setpoints.next();
        double accelM_S2 = m_setpoint.a();
        PEffort t = m_dynamics.effort(new PAcceleration(accelM_S2));

        final double position = getPosition();
        final double velocity = getVelocity();
        final ModelR1 measurement = new ModelR1(position, velocity);

        final double u_FF = m_kV * m_setpoint.v() + m_kT * t.f();
        final double u_FB = m_feedback.calculate(measurement, setpoints.current().model());
        final double u_TOTAL = MathUtil.clamp(u_FF + u_FB, -1.0, 1.0);

        m_mechanism.setDutyCycle(u_TOTAL);

        m_log_setpoint.log(() -> m_setpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpoints.current().x() - position);
        m_log_velocity_error.log(() -> setpoints.current().v() - velocity);
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
    public boolean atSetpoint() {
        double pos = m_mechanism.getPositionM();
        double vel = m_mechanism.getVelocityM_S();
        double pErr = m_setpoint.x() - pos;
        double vErr = m_setpoint.v() - vel;
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
        //
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_log_position.log(() -> getPosition());
        m_log_velocity.log(() -> getVelocity());
    }

}
