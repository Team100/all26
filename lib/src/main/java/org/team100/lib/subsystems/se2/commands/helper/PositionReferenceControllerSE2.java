package org.team100.lib.subsystems.se2.commands.helper;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

/**
 * Actuates a positional subsystem based on a positional reference.
 * 
 * Uses **outboard** positional control, so there's no controller here, it
 * just passes through the "next" reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class PositionReferenceControllerSE2 {
    private final PositionSubsystemSE2 m_subsystem;
    private final double m_tolerance;
    private final BooleanLogger m_logDone;
    private final DoubleLogger m_logToGo;
    private final ReferenceSE2 m_reference;
    private final StateSE2Logger m_log_measurement;
    private final StateSE2Logger m_log_current;
    private final ControlSE2Logger m_log_next;
    private final StateSE2Logger m_log_error;

    /**
     * Call this from Command.initialize().
     */
    public PositionReferenceControllerSE2(
            LoggerFactory parent,
            PositionSubsystemSE2 subsystem,
            double tolerance,
            ReferenceSE2 reference) {
        LoggerFactory log = parent.type(this);
        m_subsystem = subsystem;
        m_tolerance = tolerance;
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
            StateSE2 measurement = m_subsystem.getState();
            StateSE2 current = m_reference.current();
            ControlSE2 next = m_reference.next();
            StateSE2 error = current.minus(measurement);
            m_subsystem.set(next);
            m_log_measurement.log(() -> measurement);
            m_log_current.log(() -> current);
            m_log_next.log(() -> next);
            m_log_error.log(() -> error);
            isDone();
            toGo();
        } catch (IllegalStateException ex) {
            // This happens when the trajectory generator produces an empty trajectory.
        }
    }

    /**
     * Trajectory is complete, and measurement error is within tolerance.
     * 
     * Use this with "until()" to end the command.
     */
    public boolean isDone() {
        boolean done = m_reference.done() && toGo() < m_tolerance;
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
