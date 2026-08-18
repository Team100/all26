package org.team100.frc2025.Climber;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.r1.PIDFeedback;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.sim.SimulatedRotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OnboardAngularPositionServo;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber2025 extends SubsystemBase {

    private final AngularPositionServo m_servo;

    public Climber2025(LoggerFactory parent, TotalCurrentLog currentLog, CanId canID) {
        LoggerFactory log = parent.type(this);
        // dynamics are unimportant for the climber
        RDynamicsAnalytic dyn = new RDynamicsAnalytic(0, 0, 0, 0);
        ProfileR1 profile = new TrapezoidProfileR1(1, 2, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        PIDFeedback feedback = new PIDFeedback(log, 5, 0, 0, false, 0.05, 0.1);

        switch (Identity.instance) {
            case COMP_BOT -> {
                Falcon500Motor motor = new Falcon500Motor(
                        log, currentLog, canID, NeutralMode100.BRAKE, MotorPhase.REVERSE,
                        new CurrentLimit(20, 20),
                        new Friction(0.100, 0.065, 0.0, 0.5),
                        PIDConstants.makePositionPID(0.2));

                double inputOffset = 0.440602;
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        log, new RoboRioChannel(0), inputOffset, EncoderDrive.DIRECT);
                double gearRatio = 5 * 5 * 4 * 20;

                RotaryMechanism rotaryMechanism = new RotaryMechanism(
                        log, motor, sensor, gearRatio,
                        0, Math.PI / 2);

                m_servo = new OnboardAngularPositionServo(log, rotaryMechanism, dyn, ref, feedback);
            }

            default -> {
                SimulatedBareMotor climberMotor = new SimulatedBareMotor(log, 600);

                IncrementalBareEncoder encoder = climberMotor.encoder();
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(log, encoder, 1);

                RotaryMechanism climberMech = new RotaryMechanism(
                        log, climberMotor, sensor, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

                m_servo = new OnboardAngularPositionServo(log, climberMech, dyn, ref, feedback);
            }
        }
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    public boolean atGoal() {
        return m_servo.atGoal();
    }

    public double angle() {
        return m_servo.getWrappedPositionRad();
    }

    public void stopMotor() {
        m_servo.stop();
    }

    // COMMANDS

    public Command stop() {
        return run(
                () -> setDutyCycle(0));
    }

    public Command manual(DoubleSupplier s) {
        return runEnd(
                () -> setDutyCycle(s.getAsDouble()),
                () -> setDutyCycle(0));
    }

    /** Push the climber out into the intake position. */
    public Command goToIntakePosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(Math.PI / 2));
    }

    /** Pull the climber in all the way to the climb position. */
    public Command goToClimbPosition() {
        return startRun(
                () -> reset(),
                () -> setAngle(0));
    }

    ////////////////////////////////

    private void reset() {
        m_servo.reset();
    }

    private void setDutyCycle(double dutyCycle) {
        m_servo.setDutyCycle(dutyCycle);
    }

    /** Use a profile to set the position. */
    private void setAngle(double value) {
        m_servo.setPositionProfiled(value);
    }
}
