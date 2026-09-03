package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

/**
 * Bare-motor version of the discus mechanism. Just takes
 * duty cycle input.
 */
public class DiscusBare extends SubsystemBase {
    private static final double SCALE = 0.05;
    private static final double VOLT_SCALE = 0.2;
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;
    private final Motor m_motor;
    private final RotaryPositionSensor m_sensor;

    public DiscusBare(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);
        switch (Identity.instance) {
            case TEAM100_2018, SWERVE_TWO, TEST_BOARD_B0 -> {
                Friction friction = new Friction(0.14, 0.14, 0, 0);
                PIDConstants pid = PIDConstants.makePositionPID(0.0);
                m_motor = new Falcon500Motor(
                        logger,
                        currentLog,
                        new CanId(36),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        friction,
                        pid);

            }
            default -> {
                m_motor = new SimulatedMotor(logger, 600);

            }
        }
        m_sensor = new ProxyRotaryPositionSensor(m_motor.encoder(), 1.0, 0.0);
    }

    public double getPosition() {
        return m_sensor.getWrappedPositionRad();
    }

    private void setDutyCycle(double p) {
        m_motor.setDutyCycle(p);
    }

    private void setVoltage(double v) {
        m_motor.setVoltage(v);
    }

    private void setCurrent(double i) {
        m_motor.setCurrent(i);
    }

    public Command dutyCycle(DoubleSupplier p) {
        return run(() -> setDutyCycle(
                SCALE * p.getAsDouble()));
    }

    public Command voltage(DoubleSupplier v) {
        return run(() -> setVoltage(
                v.getAsDouble()));
    }

    public Command current(DoubleSupplier i) {
        return run(()->setCurrent(i.getAsDouble()));
    }

    @Override
    public void periodic() {
        m_motor.periodic();
        m_sensor.periodic();
    }

}
