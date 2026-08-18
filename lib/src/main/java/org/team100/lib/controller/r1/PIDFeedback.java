package org.team100.lib.controller.r1;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.controller.PIDController;

/**
 * Feedback using the WPI PID controller class.
 * 
 * This also logs the error that we have habitually logged elsewhere, because I
 * don't want the idea of "error" to be in the interface.
 */
public class PIDFeedback implements FeedbackR1 {
    private final PIDController m_controller;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_errorD;

    public PIDFeedback(
            LoggerFactory parent,
            double p,
            double i,
            double d,
            boolean rotation,
            double tolerance,
            double integratorRange) {
        m_controller = new PIDController(p, i, d, TimedRobot100.LOOP_PERIOD_S);
        m_controller.setTolerance(tolerance);
        m_controller.setIntegratorRange(-integratorRange, integratorRange);

        if (rotation)
            m_controller.enableContinuousInput(-Math.PI, Math.PI);
        LoggerFactory log = parent.type(this);
        m_log_error = log.doubleLogger(Level.DEBUG, "error");
        m_log_errorD = log.doubleLogger(Level.DEBUG, "errorD");

    }

    /**
     * Observe position error, produce PID output.
     */
    @Override
    public double calculate(ModelR1 measurement, ModelR1 setpoint) {
        // System.out.printf("PIDFeedback measurement %s setpoint %s\n", measurement,
        // setpoint);
        double u = m_controller.calculate(measurement.x(), setpoint.x());
        m_log_error.log(m_controller::getError);
        m_log_errorD.log(m_controller::getErrorDerivative);
        return u;
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_controller.atSetpoint();
        // System.out.printf("PIDFeedback at setpoint %b\n", atSetpoint);
        return atSetpoint;
    }

    @Override
    public void reset() {
        m_controller.reset();
    }

    @Override
    public boolean handlesWrapping() {
        return m_controller.isContinuousInputEnabled();
    }

}
