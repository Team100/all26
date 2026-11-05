package org.team100.lib.motor.ctre;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

public class Talon6Encoder implements IncrementalEncoder {
    private final Talon6Motor m_motor;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    public Talon6Encoder(LoggerFactory parent, Talon6Motor motor) {
        LoggerFactory log = parent.type(this);
        m_motor = motor;
        m_log_position = log.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (rad_s)");
        LogPoller.register(this::log);
    }

    /**
     * Latency-compensated, represents the current Takt.
     */
    @Override
    public double getUnwrappedPositionRad() {
        return m_motor.m_position.getAsDouble();
    }

    @Override
    public double getVelocityRad_S() {
        return m_motor.m_velocity.getAsDouble();
    }

    @Override
    public double getAccelerationRad_S2() {
        return m_motor.m_acceleration.getAsDouble();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Set integrated sensor position in radians.
     * 
     * Note this takes **FOREVER**, like tens of milliseconds, so you can only do it
     * at startup.
     */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        System.out.println("WARNING: Setting CTRE encoder position is very slow!");
        Talon6Motor.warn(() -> m_motor.m_motor.setPosition(positionRad / (2.0 * Math.PI), 1));
    }

    private void log() {
        m_log_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

}
