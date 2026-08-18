package org.team100.lib.motor.ctre;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.util.CanId;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Any motor connected to the Talon SRX controller.
 * 
 * As implemented here, this controller does not support sensing.
 */
public class TalonSRXMotor implements BareMotor {
    private static final double FF_DUTY_RAD_S = 0.0016;

    private final LoggerFactory m_log;
    private final TalonSRX m_motor;
    private final DoubleLogger m_log_supply;
    private final DoubleLogger m_log_stator;
    private final DoubleLogger m_log_duty;

    public TalonSRXMotor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canID,
            MotorPhase phase,
            NeutralMode100 neutral,
            double supplyLimit) {
        currentLog.register(this);
        m_motor = new TalonSRX(canID.id);
        switch (neutral) {
            case COAST -> m_motor.setNeutralMode(
                    com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
            case BRAKE -> m_motor.setNeutralMode(
                    com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        }
        switch (phase) {
            case FORWARD -> m_motor.setInverted(false);
            case REVERSE -> m_motor.setInverted(true);
        }
        // don't use the "peak" current limit feature at all
        m_motor.configPeakCurrentLimit(0);
        // the supply limit is really an input power limit; the available torque thus
        // varies with RPM.
        m_motor.configContinuousCurrentLimit((int) supplyLimit);
        m_motor.enableCurrentLimit(true);
        m_log = parent.type(this);
        m_log_supply = m_log.doubleLogger(Level.TRACE, "supply current (A)");
        m_log_stator = m_log.doubleLogger(Level.TRACE, "stator current (A)");
        m_log_duty = m_log.doubleLogger(Level.TRACE, "duty cycle");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public void setVoltage(double volts) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVelocity(double velocityRad_S, double torqueNm) {
        final double motorDutyCycle = velocityRad_S * FF_DUTY_RAD_S;
        setDutyCycle(motorDutyCycle);
    }

    @Override
    public double kROhms() {
        // this is the number for a CIM; if you use this for any other motor, you should
        // adjust it.
        return 0.09;
    }

    @Override
    public double kTNm_amp() {
        // this is the number for a CIM; if you use this for any other motor, you should
        // adjust it.
        return 0.018;
    }

    @Override
    public double kFreeSpeedRPM() {
        // Adjust this for whatever is attached.
        return 6000;
    }

    @Override
    public void stop() {
        m_motor.neutralOutput();
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        // SRX doesn't support close()
    }

    @Override
    public void periodic() {
        m_log_supply.log(m_motor::getSupplyCurrent);
        m_log_stator.log(m_motor::getStatorCurrent);
        m_log_duty.log(m_motor::getMotorOutputPercent);
    }

    @Override
    public double getCurrent() {
        return m_motor.getStatorCurrent();
    }

    @Override
    public double getSupplyCurrent() {
        return m_motor.getSupplyCurrent();
    }

    // unsupported methods

    @Override
    public IncrementalBareEncoder encoder() {
        throw new UnsupportedOperationException("TalonSRX sensing is not supported.");
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        throw new UnsupportedOperationException("TalonSRX limits power, not torque.");
    }

    @Override
    public double getVelocityRad_S() {
        throw new UnsupportedOperationException("TalonSRX sensing is not supported.");
    }

    @Override
    public double getUnwrappedPositionRad() {
        throw new UnsupportedOperationException("TalonSRX sensing is not supported.");
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        throw new UnsupportedOperationException("TalonSRX sensing is not supported.");
    }

    @Override
    public void setUnwrappedPosition(double positionRad, double velocityRad_S, double torqueNm) {
        throw new UnsupportedOperationException("TalonSRX sensing is not supported.");
    }

    @Override
    public void play(double freq) {
    }

}
