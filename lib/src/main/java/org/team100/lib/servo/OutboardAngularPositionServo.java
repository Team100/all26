package org.team100.lib.servo;

import org.team100.lib.dynamics.r.RAcceleration;
import org.team100.lib.dynamics.r.RConfig;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.dynamics.r.REffort;
import org.team100.lib.dynamics.r.RVelocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.SetpointsR1Logger;
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
    private static final boolean DEBUG = false;
    private final SetpointsR1Logger m_log_setpoints;
    private final DoubleLogger m_log_ff_torque;
    private final DoubleLogger m_log_position_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_log_accel_error;

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            RDynamics dynamics,
            ReferenceR1 ref,
            double xtolerance,
            double vtolerance) {
        super(parent, mech, dynamics, ref, xtolerance, vtolerance);
        LoggerFactory log = parent.type(this);
        m_log_setpoints = log.setpointsR1Logger(Level.TRACE, "setpoints");
        m_log_ff_torque = log.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_position_error = log.doubleLogger(Level.COMP, "position error (rad)");
        m_log_velocity_error = log.doubleLogger(Level.COMP, "velocity error (rad_s)");
        m_log_accel_error = log.doubleLogger(Level.COMP, "accel error (rad_s2)");
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
        return new OutboardAngularPositionServo(
                log, mech, dyn, ref, 0.02, 0.02);
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
        return new OutboardAngularPositionServo(
                log, mech, dyn, ref, 0.02, 0.02);
    }

    /**
     * Pass the next setpoint directly to the mechanism's position controller.
     * Ignores current setpoint. We only use the "next" setpoint.
     */
    @Override
    void actuate(SetpointsR1 unwrappedSetpoint) {
        if (DEBUG)
            System.out.printf("actuate %s\n", unwrappedSetpoint);
        ControlR1 nextUnwrappedSetpoint = unwrappedSetpoint.next();

        REffort t = m_dynamics.effort(
                new RConfig(nextUnwrappedSetpoint.x()),
                new RVelocity(nextUnwrappedSetpoint.v()),
                new RAcceleration(nextUnwrappedSetpoint.a()));

        m_mechanism.setUnwrappedPosition(
                nextUnwrappedSetpoint.x(),
                nextUnwrappedSetpoint.v(),
                t.t());

        m_log_setpoints.log(() -> unwrappedSetpoint);
        m_log_ff_torque.log(() -> t.t());

        m_log_position_error.log(() -> unwrappedSetpoint.current().x() - getUnwrappedPositionRad());
        m_log_velocity_error.log(() -> unwrappedSetpoint.current().v() - getVelocity());
        m_log_accel_error.log(() -> unwrappedSetpoint.current().a() - getAcceleration());
    }

}
