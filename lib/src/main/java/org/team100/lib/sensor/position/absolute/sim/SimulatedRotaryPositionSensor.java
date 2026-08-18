package org.team100.lib.sensor.position.absolute.sim;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

import edu.wpi.first.math.MathUtil;

/**
 * Integrates encoder velocity to find position. It repeats the gear ratio
 * that's in the mechanism, to avoid a circular dependency.
 * 
 * If you use this in tests, you'll have to control the clock somehow, e.g. by
 * using {@link Timeless}.
 * 
 * TODO: I think this can be replaced by ProxyRotaryPositionSensor.
 */
public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final IncrementalBareEncoder m_encoder;
    private final double m_gearRatio;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_rate;

    private double m_positionRad = 0;
    // to calculate the position with trapezoid integral
    private double m_previousVelocity = 0;
    private double m_timeS = Takt.get();

    public SimulatedRotaryPositionSensor(
            LoggerFactory parent,
            IncrementalBareEncoder encoder,
            double gearRatio) {
        LoggerFactory log = parent.type(this);
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_log_position = log.doubleLogger(Level.TRACE, "position");
        m_log_rate = log.doubleLogger(Level.TRACE, "rate");
    }

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(getUnwrappedPositionRad());
    }

    @Override
    public double getUnwrappedPositionRad() {
        updatePosition();
        return m_positionRad;
    }

    @Override
    public double getVelocityRad_S() {
        double m_rate = encoderVelocityRad_S();
        m_log_rate.log(() -> m_rate);
        return m_rate;
    }

    @Override
    public void periodic() {
        m_encoder.periodic();
        m_log_position.log(() -> m_positionRad);
    }

    @Override
    public void close() {
        //
    }

    ///////////////////////////////////////////////////////////

    /** The same as RotaryMechanism.getVelocityRad_S(). */
    private double encoderVelocityRad_S() {
        return m_encoder.getVelocityRad_S() / m_gearRatio;
    }

    /**
     * Integrates the mechanism velocity between the previous call and the current
     * instant.
     */
    private void updatePosition() {
        double nowS = Takt.get();
        double dtS = nowS - m_timeS;
        if (dtS > 0.04) {
            // clock is unreliable, skip the update
            dtS = 0;
        }
        // this is the velocity at the current instant.
        // motor velocity is rad/s
        double velocityRad_S = encoderVelocityRad_S();

        // use the previous velocity to calculate the trapezoidal integral
        m_positionRad += 0.5 * (velocityRad_S + m_previousVelocity) * dtS;
        m_previousVelocity = velocityRad_S;

        m_timeS = nowS;
    }

}
