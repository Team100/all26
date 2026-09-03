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
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

/**
 * Actuates a *velocity* subsystem based on a *positional* reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class VelocityReferenceControllerSE2 {
    private final VelocitySubsystemSE2 m_subsystem;
    private final BooleanLogger m_logDone;
    private final DoubleLogger m_logToGo;
    private final ControllerSE2 m_controller;
    private final ReferenceSE2 m_reference;
    private final StateSE2Logger m_log_measurement;
    private final StateSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final StateSE2Logger m_log_error;

    /**
     * Call this from Command.initialize().
     */
    public VelocityReferenceControllerSE2(
            LoggerFactory parent,
            VelocitySubsystemSE2 subsystem,
            ControllerSE2 controller,
            ReferenceSE2 reference) {
        LoggerFactory log = parent.type(this);
        m_subsystem = subsystem;
        m_controller = controller;
        m_reference = reference;
        m_logDone = log.booleanLogger(Level.TRACE, "done");
        m_logToGo = log.doubleLogger(Level.TRACE, "to go");
        m_log_measurement = log.StateSE2Logger(Level.TRACE, "measurement");
        m_log_current = log.StateSE2Logger(Level.TRACE, "current");
        m_log_next = log.controlSE2Logger(Level.TRACE, "next");
        m_log_error = log.StateSE2Logger(Level.TRACE, "error");
        // initialize here so that the "done" state knows about the clock
        m_reference.initialize(subsystem.getState());
    }

    /**
     * This should be called in Command.execute().
     */
    public void execute() {
        try {
            // TODO: control noise in this measurement
            StateSE2 measurement = m_subsystem.getState();
            StateSE2 current = m_reference.current();
            ControlSE2 next = m_reference.next();
            StateSE2 error = current.minus(measurement);
            // u represents the time from now until now+dt, so it's also
            // what the mechanism should be doing at the next time step
            VelocityControlSE2 u = m_controller.calculate(
                    measurement, current, next);
            m_subsystem.set(u);
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
