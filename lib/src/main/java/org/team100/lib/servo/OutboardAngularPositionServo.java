package org.team100.lib.servo;

import org.team100.lib.dynamics.r.RAcceleration;
import org.team100.lib.dynamics.r.RConfig;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.dynamics.r.REffort;
import org.team100.lib.dynamics.r.RVelocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ControlR1Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.state.ControlR1;

/**
 * Uses mechanism position control.
 * 
 * Uses a profile with velocity feedforward, also extra torque (e.g. for
 * gravity). There's no feedback at this level, and no feedforward calculation
 * either: the mechanism does that.
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder so that
 * positional commands make sense.
 */
public class OutboardAngularPositionServo extends AngularPositionServoImpl {

    private final DoubleLogger m_log_ff_torque;
    private final ControlR1Logger m_log_control;

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            RDynamics dynamics,
            ReferenceR1 ref) {
        super(parent, mech, dynamics, ref);
        LoggerFactory log = parent.type(this);
        m_log_ff_torque = log.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_control = log.ControlR1Logger(Level.TRACE, "setpoint (rad)");
    }

    /**
     * Make a servo from a motor and a position reference.
     * Creates the mechanism in between.
     */
    public static OutboardAngularPositionServo make(
            LoggerFactory log,
            Motor motor,
            RDynamicsAnalytic dyn,
            ReferenceR1 ref,
            double gearRatio,
            double initialPosition) {
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, motor.encoder(), initialPosition, gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardAngularPositionServo(log, mech, dyn, ref);
    }

    /**
     * Make a servo from a motor and a position reference.
     * Creates the mechanism in between.
     */
    public static OutboardAngularPositionServo make(
            LoggerFactory log,
            Motor motor,
            RDynamics dyn,
            ReferenceR1 ref,
            double gearRatio,
            double initialPosition,
            double minPosition,
            double maxPosition) {
        RotaryMechanism mech = new RotaryMechanism(
                log, motor, motor.encoder(), initialPosition, gearRatio,
                minPosition, maxPosition);
        return new OutboardAngularPositionServo(log, mech, dyn, ref);
    }

    /**
     * Pass the next setpoint directly to the mechanism's position controller.
     * Ignores current setpoint. We only use the "next" setpoint.
     */
    @Override
    void actuate(SetpointsR1 unwrappedSetpoint) {

        ControlR1 nextUnwrappedSetpoint = unwrappedSetpoint.next();

        REffort t = m_dynamics.effort(
                new RConfig(nextUnwrappedSetpoint.x()),
                new RVelocity(nextUnwrappedSetpoint.v()),
                new RAcceleration(nextUnwrappedSetpoint.a()));

        m_mechanism.setUnwrappedPosition(
                nextUnwrappedSetpoint.x(),
                nextUnwrappedSetpoint.v(),
                t.t());

        m_log_control.log(() -> nextUnwrappedSetpoint);
        m_log_ff_torque.log(() -> t.t());
    }

}
