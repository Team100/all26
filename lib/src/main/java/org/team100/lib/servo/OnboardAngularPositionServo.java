package org.team100.lib.servo;

import org.team100.lib.controller.r1.FeedbackR1;
import org.team100.lib.dynamics.r.RAcceleration;
import org.team100.lib.dynamics.r.RConfig;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.dynamics.r.REffort;
import org.team100.lib.dynamics.r.RVelocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateR1Logger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

/**
 * Uses mechanism velocity control.
 * 
 * Uses a profile with velocity feedforward, feedback here in Java-land, and
 * extra torque (e.g. for gravity).
 */
public class OnboardAngularPositionServo extends AngularPositionServoImpl {
    private static final boolean DEBUG = false;

    private final FeedbackR1 m_feedback;

    private final DoubleLogger m_log_feedforward_torque;
    private final StateR1Logger m_log_measurement;
    private final ControlR1Logger m_log_control;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            RDynamics dynamics,
            ReferenceR1 ref,
            FeedbackR1 feedback) {
        super(parent, mech, dynamics, ref);
        if (feedback.handlesWrapping())
            throw new IllegalArgumentException("Do not supply wrapping feedback");
        LoggerFactory log = parent.type(this);
        m_feedback = feedback;

        m_log_feedforward_torque = log.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = log.StateR1Logger(Level.COMP, "measurement (rad)");
        m_log_control = log.ControlR1Logger(Level.COMP, "control (rad)");
        m_log_u_FB = log.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = log.doubleLogger(Level.TRACE, "u_FF (rad_s)");

        m_log_u_TOTAL = log.doubleLogger(Level.COMP, "u_TOTAL (rad_s)");
        m_log_error = log.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = log.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
    }

    @Override
    public void reset() {
        super.reset();
        m_feedback.reset();
    }

    /**
     * Feedback using measurement and current setpoint. Feedforward using next
     * setpoint.
     */
    @Override
    void actuate(SetpointsR1 unwrappedSetpoint) {
        if (DEBUG) {
            System.out.printf("setpoint %s\n", unwrappedSetpoint);
        }

        StateR1 unwrappedMeasurement = m_mechanism.getUnwrappedMeasurement();
        StateR1 currentUnwrappedSetpoint = unwrappedSetpoint.current().model();
        ControlR1 nextUnwrappedSetpoint = unwrappedSetpoint.next();

        REffort t = m_dynamics.effort(
                new RConfig(nextUnwrappedSetpoint.x()),
                new RVelocity(nextUnwrappedSetpoint.v()),
                new RAcceleration(nextUnwrappedSetpoint.a()));

        if (DEBUG) {
            System.out.printf("unwrapped Measurement %s currentUnwrappedSetpoint %s\n",
                    unwrappedMeasurement, currentUnwrappedSetpoint);
        }
        double u_FB = m_feedback.calculate(unwrappedMeasurement, currentUnwrappedSetpoint);
        double u_FF = nextUnwrappedSetpoint.v();
        double u_TOTAL = u_FB + u_FF;
        if (DEBUG) {
            System.out.printf("u_FB %6.3f u_FF %6.3f u_TOTAL %6.3f\n", u_FB, u_FF, u_TOTAL);
        }

        m_mechanism.setVelocity(u_TOTAL, t.t());

        m_log_feedforward_torque.log(() -> t.t());
        m_log_measurement.log(() -> unwrappedMeasurement);
        m_log_control.log(() -> nextUnwrappedSetpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> currentUnwrappedSetpoint.x() - unwrappedMeasurement.x());
        m_log_velocity_error.log(() -> currentUnwrappedSetpoint.v() - unwrappedMeasurement.v());
    }

}
