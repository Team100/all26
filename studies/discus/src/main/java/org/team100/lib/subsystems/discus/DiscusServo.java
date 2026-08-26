package org.team100.lib.subsystems.discus;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.ProxyRotaryPositionSensor;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

/**
 * Discus version that uses the "servo" abstraction, which
 * provides profiled motion control.
 */
public class DiscusServo extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double VELOCITY_TOLERANCE = 0.05;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 100;
    private static final double STATOR_LIMIT = 100;
    private static final double MAX_VELOCITY = 10; // rad/s
    private static final double MAX_ACCEL = 20; // rad/s/s

    private final ProxyRotaryPositionSensor m_sensor;
    private final AngularPositionServo m_servo;

    public DiscusServo(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory logger = parent.type(this);

        // zeros
        // PID = 1.0, 0.0, 0.05
        PIDConstants pid = PIDConstants.makePositionPID(0.5, 0, 0.05); // d = 0.12 was experimentally found
        Friction friction = new Friction(0.15, 0.14, 0, 0);
        ProfileR1 profile = new TrapezoidProfileR1(
                MAX_VELOCITY, MAX_ACCEL, POSITION_TOLERANCE);
        ReferenceR1 ref = new ProfileReferenceR1(
                logger, () -> profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        RDynamics dyn = new RDynamicsAnalytic(0, 0, 0, 0.005);

        BareMotor motor;
        switch (Identity.instance) {
            case TEAM100_2018 -> {
                motor = new Falcon500Motor(
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
                motor = new SimulatedBareMotor(logger, 600);
            }
        }
        m_sensor = new ProxyRotaryPositionSensor(motor.encoder(), 1.0);

        RotaryMechanism mech = new RotaryMechanism(
                logger,
                motor,
                m_sensor,
                1.0,
                -100.0,
                100.0);

        m_servo = new OutboardAngularPositionServo(
                logger,
                mech,
                dyn,
                ref);

    }

    public void setPosition(double p) {
        // m_servoP1.setPositionProfiled(p1, 0);
        m_servo.actuateWithProfile(p);
    }

    public double getPosition() {
        return m_servo.getUnwrappedPositionRad();
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    //////////////////////

    private void setDutyCycle(double p) {
        m_servo.setDutyCycle(p);
    }

    private void resetEncoderPosition() {
        m_sensor.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.05));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    public Command position(DoubleSupplier p) {
        return run(() -> setPosition(p.getAsDouble()));
    }
}
