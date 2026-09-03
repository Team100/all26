package org.team100.lib.motor.wpi;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.Motor;
import org.team100.lib.sensor.position.incremental.IncrementalEncoder;
import org.team100.lib.sensor.position.incremental.sim.SimulatedEncoder;
import org.wpilib.hardware.motor.MotorController;

/** Wrapoer for RoboRIO-connected PWM speed control */
public class MotorController100 implements Motor {
    private static final double FREE_SPEED_RPM = 6000;
    /**
     * Very much not calibrated.
     * Say 600 rad/s max so 0.0016?
     */
    private static final double velocityFFDutyCycle_Rad_S = 0.0016;
    private final LoggerFactory m_log;
    private final MotorController m_motor;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_reported;

    public MotorController100(
            LoggerFactory parent,
            MotorController motorController) {
        m_log = parent.type(this);
        m_log_duty = m_log.doubleLogger(Level.TRACE, "duty cycle");
        m_log_reported = m_log.doubleLogger(Level.TRACE, "duty cycle reported");
        m_motor = motorController;
        m_motor.setInverted(true);
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.setThrottle(output);
        m_log_duty.log(() -> output);
    }

    @Override
    public void setVoltage(double volts) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setCurrent(double current) {
        throw new UnsupportedOperationException();
    }

    /**
     * Open-loop velocity control using velocity feedforward only.
     */
    @Override
    public void setVelocity(double motorRad_S, double torqueNm) {
        final double motorDutyCycle = motorRad_S * velocityFFDutyCycle_Rad_S;
        m_motor.setThrottle(motorDutyCycle);
        m_log_duty.log(() -> motorDutyCycle);
    }

    /** MotorControllers do not support positional control. */
    @Override
    public void setUnwrappedPosition(double position, double velocity, double torque) {
        throw new UnsupportedOperationException();
    }

    /** placeholder */
    @Override
    public double R() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kT() {
        return 0.02;
    }

    @Override
    public double kE() {
        return 60 * 12 / (FREE_SPEED_RPM * 2 * Math.PI);
    }

    public IncrementalEncoder encoder() {
        return new SimulatedEncoder(m_log, this);
    }

    @Override
    public void stop() {
        // m_motor.stopMotor();
        m_motor.setThrottle(0);
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        // m_motor.close();
    }

    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getAccelerationRad_S2() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getUnwrappedPositionRad() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getStatorCurrent() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void periodic() {
        m_log_reported.log(m_motor::getThrottle);
    }

    @Override
    public void play(double freq) {
    }

    @Override
    public double getSupplyCurrent() {
        // no current measurement
        return 0;
    }
}
