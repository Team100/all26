package org.team100.lib.subsystems.se2.commands.helper;

import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.SubsystemSE2;

/**
 * TODO: make this a component of the subclasses, not a parent
 */
public abstract class ReferenceControllerSE2Base {
    private final BooleanLogger m_logDone;
    private final DoubleLogger m_logToGo;
    private final SubsystemSE2 m_subsystem;
    private final ControllerSE2 m_controller;
    private final ReferenceSE2 m_reference;
    private final StateSE2Logger m_log_measurement;
    private final StateSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final StateSE2Logger m_log_error;

    /**
     * Initializes the reference with the current measurement.
     */
    ReferenceControllerSE2Base(
            LoggerFactory parent,
            SubsystemSE2 subsystem,
            ControllerSE2 controller,
            ReferenceSE2 reference) {
        LoggerFactory log = parent.type(this);
        m_logDone = log.booleanLogger(Level.TRACE, "done");
        m_logToGo = log.doubleLogger(Level.TRACE, "to go");
        m_subsystem = subsystem;
        m_controller = controller;
        m_reference = reference;
        m_log_measurement = log.StateSE2Logger(Level.TRACE, "measurement");
        m_log_current = log.StateSE2Logger(Level.TRACE, "current");
        m_log_next = log.controlSE2Logger(Level.TRACE, "next");
        m_log_error = log.StateSE2Logger(Level.TRACE, "error");
        // initialize here so that the "done" state knows about the clock
        m_reference.initialize(subsystem.getState());
    }

    /**
     * Actuate the subsystem here.
     * 
     * TODO: this seems like a confusing mess; one implementation uses one argument,
     * the other one uses the other argument.
     * 
     * @param next The next control setpoint.
     * @param u    The controller output for the next dt
     */
    abstract void execute100(ControlSE2 next, VelocityControlSE2 u);

    /**
     * This should be called in Command.execute().
     */
    public void execute() {
        try {
            StateSE2 measurement = m_subsystem.getState();
            StateSE2 current = m_reference.current();
            ControlSE2 next = m_reference.next();
            StateSE2 error = current.minus(measurement);
            // u represents the time from now until now+dt, so it's also
            // what the mechanism should be doing at the next time step
            VelocityControlSE2 u = m_controller.calculate(
                measurement, current, next);
            execute100(next, u);
            m_log_measurement.log(() -> measurement);
            m_log_current.log(() -> current);
            m_log_next.log(() -> next);
            m_log_error.log(() -> error);
            // log these
            isDone();
            toGo();
        } catch (IllegalStateException ex) {
            // This happens when the trajectory generator produces an empty trajectory.
        }
    }

    /**
     * Trajectory is complete, and controller error is within tolerance.
     * 
     * Use this with "until()" to end the command.
     */
    public boolean isDone() {
        boolean done = m_reference.done() && m_controller.atReference();
        m_logDone.log(() -> done);
        return done;
    }

    /**
     * Distance between the measurement and the goal.
     * 
     * Use this to start, or finish, parallel commands while this one is still
     * running, e.g. "start doing X if Y is near the goal."
     */
    public double toGo() {
        StateSE2 goal = m_reference.goal();
        StateSE2 measurement = m_subsystem.getState();
        double togo = goal.minus(measurement).translation().getNorm();
        m_logToGo.log(() -> togo);
        return togo;
    }

}
