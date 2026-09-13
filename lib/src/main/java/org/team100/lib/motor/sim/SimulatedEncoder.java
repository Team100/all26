package org.team100.lib.motor.sim;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;

public class SimulatedEncoder implements IncrementalEncoder {
    private static final boolean DEBUG = false;
    private final SimulatedMotor m_motor;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_accel;

    // TODO: move the encoder and motor to the same package.
    public double m_offset;

    public SimulatedEncoder(
            LoggerFactory parent,
            SimulatedMotor motor) {
        LoggerFactory log = parent.type(this);
        m_motor = motor;
        m_offset = 0;
        m_log_position = log.doubleLogger(Level.TRACE, "position (rad)");
        m_log_velocity = log.doubleLogger(Level.TRACE, "velocity (rad_s)");
        m_log_accel = log.doubleLogger(Level.TRACE, "accel (rad_s2)");
    }

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Derives position by integrating velocity over one time step.
     */
    @Override
    public double getUnwrappedPositionRad() {
        // TODO: move the encoder and motor to the same package.

        // offset is subtracted in order to be consistent with the roborio sensor
        double positionRad = m_motor.m_stateCache.get().x() - m_offset;
        if (Double.isNaN(positionRad))
            throw new IllegalArgumentException("motor pos");
        if (DEBUG) {
            System.out.printf("read encoder position %.6f\n", positionRad);
        }
        return positionRad;
    }

    @Override
    public double getVelocityRad_S() {
        return m_motor.m_stateCache.get().v();
    }

    @Override
    public double getAccelerationRad_S2() {
        // this is computed in update
        return m_motor.m_smoothDerivative.lastValue();
    }

    @Override
    public void close() {
        //
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        if (Double.isNaN(positionRad))
            throw new IllegalArgumentException("motor set position");

        // this now just affects the offset (!)
        // the ground-truth position does not change
        double gt = m_motor.m_stateCache.get().x();
        m_offset = gt - positionRad;

        // TODO: move the motor and encoder into the same package so this can be
        // package-private.
        // m_motor.m_state = new StateR1(positionRad, 0);
        // m_motor.m_stateCache.reset();
    }

    @Override
    public void periodic() {
        m_log_position.log(this::getUnwrappedPositionRad);
        m_log_velocity.log(this::getVelocityRad_S);
        m_log_accel.log(this::getAccelerationRad_S2);
    }
}
