package org.team100.lib.sensor.position.incremental.sim;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

public class SimulatedBareEncoder implements IncrementalBareEncoder {
    private static final boolean DEBUG = false;
    private final BareMotor m_motor;

    private DoubleLogger m_log_position;
    private DoubleLogger m_log_velocity;

    public SimulatedBareEncoder(
            LoggerFactory parent,
            BareMotor motor) {
        LoggerFactory log = parent.type(this);
        m_motor = motor;
        m_log_position = log.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (rad_s)");
    }

    /** Value should be updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return m_motor.getVelocityRad_S();
    }

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Derives position by integrating velocity over one time step.
     */
    @Override
    public double getUnwrappedPositionRad() {
        double positionRad = m_motor.getUnwrappedPositionRad();
        if (DEBUG) {
            System.out.printf("read encoder position %.6f\n", positionRad);
        }
        return positionRad;
    }

    @Override
    public void close() {
        //
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
