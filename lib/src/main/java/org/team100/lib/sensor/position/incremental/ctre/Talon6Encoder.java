package org.team100.lib.sensor.position.incremental.ctre;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.ctre.Talon6Motor;
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
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_motor.getUnwrappedPositionRad();
    }

    @Override
    public double getVelocityRad_S() {
        return m_motor.getVelocityRad_S();
    }

    @Override
    public double getAccelerationRad_S2() {
        return m_motor.getAccelerationRad_S2();
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
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
        m_motor.setUnwrappedEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }

}
