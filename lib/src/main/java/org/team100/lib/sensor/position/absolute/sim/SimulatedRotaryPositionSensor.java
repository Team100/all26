package org.team100.lib.sensor.position.absolute.sim;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

import org.wpilib.math.util.MathUtil;

/**
 * A simple proxy for IncrementalEncoder.
 */
public class SimulatedRotaryPositionSensor implements RotaryPositionSensor {
    private final IncrementalEncoder m_encoder;
    private final double m_gearRatio;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_accel;

    public SimulatedRotaryPositionSensor(
            LoggerFactory parent,
            IncrementalEncoder encoder,
            double gearRatio) {
        LoggerFactory log = parent.type(this);
        m_encoder = encoder;
        m_gearRatio = gearRatio;
        m_log_position = log.doubleLogger(Level.TRACE, "position");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity");
        m_log_accel = log.doubleLogger(Level.TRACE, "accel");
        LogPoller.register(this::log);
    }

    @Override
    public double getWrappedPositionRad() {
        return MathUtil.angleModulus(getUnwrappedPositionRad());
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_encoder.getUnwrappedPositionRad() / m_gearRatio;
    }

    @Override
    public double getVelocityRad_S() {
        return m_encoder.getVelocityRad_S() / m_gearRatio;
    }

    @Override
    public double getAccelerationRad_S2() {
        return m_encoder.getAccelerationRad_S2() / m_gearRatio;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double x) {
        m_encoder.setUnwrappedEncoderPositionRad(x);
    }

    private void log() {
        m_log_position.log(() -> getUnwrappedPositionRad());
        m_log_velocity.log(() -> getVelocityRad_S());
        m_log_accel.log(() -> getAccelerationRad_S2());
    }

    @Override
    public void close() {
        //
    }
}
