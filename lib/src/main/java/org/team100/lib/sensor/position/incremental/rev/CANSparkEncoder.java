package org.team100.lib.sensor.position.incremental.rev;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.rev.CANSparkMotor;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

/**
 * The built-in encoder in Neo motors.
 * 
 * This encoder simply senses the 14 rotor magnets in 3 places, so it's 42 ticks
 * per turn.
 */
public class CANSparkEncoder implements IncrementalEncoder {
    private final CANSparkMotor m_motor;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;

    public CANSparkEncoder(LoggerFactory parent, CANSparkMotor motor) {
        LoggerFactory log = parent.type(this);
        m_motor = motor;
        m_log_position = log.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    @Override
    public void close() {
        //
    }

    //////////////////////////////////

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
    public void setUnwrappedEncoderPositionRad(double motorPositionRad) {
        m_motor.setUnwrappedEncoderPositionRad(motorPositionRad);
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
    }
}
