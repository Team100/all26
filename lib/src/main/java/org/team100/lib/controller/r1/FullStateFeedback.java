package org.team100.lib.controller.r1;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.MathUtil;

/**
 * Patterned after FullStateDriveController.
 * 
 * Does not include feedforward, this just does feedback.
 */
public class FullStateFeedback implements FeedbackR1 {
    private static final boolean DEBUG = false;

    private final ModelR1Logger m_log_measurement;
    private final ModelR1Logger m_log_reference;
    private final ModelR1Logger m_log_error;
    private final DoubleLogger m_log_u_FB;
    private final double m_K1; // position
    private final double m_K2; // velocity
    private final boolean m_rotation;
    private final DoubleUnaryOperator m_modulus;
    private final double m_tol1;
    private final double m_tol2;

    private boolean m_atSetpoint = false;

    /**
     * @param parent   logger
     * @param k1       position gain
     * @param k2       velocity gain
     * @param rotation for rotary
     * @param xtol     for "at setpoint"
     * @param vtol     for "at setpoint"
     */
    public FullStateFeedback(
            LoggerFactory parent,
            double k1,
            double k2,
            boolean rotation,
            double xtol,
            double vtol) {
        LoggerFactory log = parent.type(this);
        m_log_reference = log.ModelR1Logger(Level.DEBUG, "reference");
        m_log_measurement = log.ModelR1Logger(Level.DEBUG, "measurement");
        m_log_error = log.ModelR1Logger(Level.DEBUG, "error");
        m_log_u_FB = log.doubleLogger(Level.DEBUG, "u_FB");
        m_K1 = k1;
        m_K2 = k2;
        m_rotation = rotation;
        m_modulus = rotation ? MathUtil::angleModulus : x -> x;
        m_tol1 = xtol;
        m_tol2 = vtol;
    }

    @Override
    public double calculate(ModelR1 measurement, ModelR1 reference) {
        m_log_measurement.log(() -> measurement);
        m_log_reference.log(() -> reference);
        m_log_error.log(() -> reference.minus(measurement));
        double u_FB = calculateFB(measurement, reference);
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    private double calculateFB(ModelR1 measurement, ModelR1 setpoint) {
        double xError = m_modulus.applyAsDouble(setpoint.x() - measurement.x());
        double xDotError = setpoint.v() - measurement.v();
        if (DEBUG)
            System.out.printf("xerr %f xdoterr %f\n", xError, xDotError);
        m_atSetpoint = Math.abs(xError) < m_tol1 && Math.abs(xDotError) < m_tol2;
        double k1 = m_K1;
        double k2 = m_K2;
        if (DEBUG)
            System.out.printf("k1 %f k2 %f\n", k1, k2);
        return k1 * xError + k2 * xDotError;
    }

    /** True if the most recent call to calculate() was at the setpoint. */
    public boolean atSetpoint() {
        return m_atSetpoint;
    }

    public void reset() {
        m_atSetpoint = false;
    }

    @Override
    public boolean handlesWrapping() {
        return m_rotation;
    }
}
