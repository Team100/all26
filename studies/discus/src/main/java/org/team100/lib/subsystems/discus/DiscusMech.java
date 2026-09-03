package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.sensor.position.absolute.HomingRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

/**
 * Discus version that uses the "mechanism" abstraction,
 * which provides positional control.
 */
public class DiscusMech extends SubsystemBase {

    /** Low current limits */
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;

    private final RotaryMechanism m_mech;

    private final Motor m_motor;

    private final HomingRotaryPositionSensor m_sensor;

    public DiscusMech(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);
        /** Units of positional PID are volts per revolution. */
        // PIDConstants pid = PIDConstants.makePositionPID(0.0); // 2.0
        PIDConstants pid = PIDConstants.makePositionPID(0.5, 0, 0.1); // 2.0

        Friction friction = new Friction(0.16, 0.15, 0, 0);

        switch (Identity.instance) {
            case TEAM100_2018 -> {
                Falcon500Motor motor = new Falcon500Motor(
                        logger,
                        currentLog,
                        new CanId(36),
                        NeutralMode100.COAST,
                        MotorPhase.REVERSE,
                        new CurrentLimit(STATOR_LIMIT, SUPPLY_LIMIT),
                        friction,
                        pid);

                m_motor = motor;

                m_sensor = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(motor.encoder(), 1.0));

                m_mech = new RotaryMechanism(
                        logger,
                        motor,
                        m_sensor,
                        1.0,
                        -100.0,
                        100.0);

            }
            default -> {
                SimulatedMotor motor = new SimulatedMotor(logger, 600);
                m_motor = motor;

                m_sensor = new HomingRotaryPositionSensor(
                        new ProxyRotaryPositionSensor(
                                motor.encoder(), 1.0));

                m_mech = new RotaryMechanism(
                        logger,
                        motor,
                        m_sensor,
                        1.0,
                        -100.0,
                        100.0);

            }
        }
    }

    /** Update position by adding. */
    public void add(double p) {
        double q1 = m_mech.getUnwrappedPositionRad() + p;
        setPosition(q1);
    }

    /** Set position goal, motionless. */
    public void setPosition(double p) {
        m_mech.setUnwrappedPosition(p, 0, 0);
    }

    public void setVelocity(double v) {
        m_mech.setVelocity(v, 0);
    }

    @Override
    public void periodic() {
        m_mech.periodic();
    }

    public double getPosition() {
        return m_mech.getWrappedPositionRad();
    }

    //////////////////////

    /** For homing; ignores feasibility and limits. */
    private void setDutyCycle(double p) {
        m_mech.setDutyCycleUnlimited(p);
    }

    /**
     * The "home" position is the max value; the idea is that you've run the duty
     * cycle (gently) to the end of travel before pushing the "home" button.
     */
    private void setHomePosition() {
        m_motor.setUnwrappedEncoderPositionRad(0);
    }

    ///////////////////////
    //
    // Commands

    /** Move in the direction of home: q1 to max, q5 to min */
    public Command home() {
        return run(() -> setDutyCycle(0.01));
    }

    /** Set the position sensor to the home position. */
    public Command zero() {
        return runOnce(this::setHomePosition);
    }

    public Command position(DoubleSupplier p1) {
        return run(() -> setPosition(p1.getAsDouble()));
    }

    public Command velocity(DoubleSupplier v) {
        return run(() -> setVelocity(v.getAsDouble()));
    }

    /** Voltage should be something like 0.1. */
    public Command friction(DoubleSupplier v) {
        return run(() -> m_mech.setVoltage(v.getAsDouble()));
    }
}
