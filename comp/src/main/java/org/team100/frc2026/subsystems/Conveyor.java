package org.team100.frc2026.subsystems;

import org.team100.frc2026.robot.CurrentLimits;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
    private static final CanId canID1 = new CanId(19);
    private static final CanId canID2 = new CanId(20);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 3;
    private static final double WHEEL_DIAMETER_M = 0.035;
    // TODO: TUNE
    private static final double NORMAL_SPEED = 5.0;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    public Conveyor(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Conveyor1");
        LoggerFactory log2 = log.name("Conveyor2");
        // equivalent linear dynamics for the actual drum inertia.
        PDynamics dynamics = PDynamics.drum(0.001, 0.025);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);

        final BareMotor m1;
        final BareMotor m2;

        switch (Identity.instance) {
            case TEST_BOARD_B0 -> {
                // friction test 3/12/262
                Friction friction = new Friction(0.7, 0.7, 0.0, 0.5);
                // tune 3/12/26
                PIDConstants pid = PIDConstants.makeVelocityPID(0.08);

                m1 = new KrakenX44Motor(
                        log1, currentLog, canID1, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.CONVEYOR, friction, pid);
                m2 = new KrakenX44Motor(
                        log2, currentLog, canID2, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.CONVEYOR, friction, pid);
            }
            default -> {
                m1 = new SimulatedBareMotor(log1, 600);
                m2 = new SimulatedBareMotor(log2, 600);
            }
        }
        m_servo1 = OutboardLinearVelocityServo.make(
                log1, m1, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo2 = OutboardLinearVelocityServo.make(
                log2, m2, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    /**
     * Use a profile to spin up the conveyor to the normal speed.
     * Never ends, but stops the motor when interrupted.
     */
    public Command convey() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(NORMAL_SPEED))
                .finallyDo(this::stopMotor)
                .withName("Convey");
    }

    /** Roll backwards to clear jams */
    public Command back() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(-5))
                .withName("Conveyor back");
    }

    public Command testConveyor() {
        return run(this::dutyCycleAll)
                .withName("Conveyor Test");
    }

    public Command testConveyorBack() {
        return run(this::dutyCycleBackAll)
                .withName("Conveyor Test Back");
    }

    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Conveyor");
    }

    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Conveyor Once");
    }

    /** For testing friction only */
    public Command setVelocity(double x) {
        return startRun(
                this::reset,
                () -> {
                    m_servo1.setVelocityProfiled(x);
                    m_servo2.setVelocityProfiled(x);
                })
                .withName("set velocity");
    }

    //////////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void setVelocityProfiled(double goalM_S) {
        m_servo1.setVelocityProfiled(goalM_S);
        m_servo2.setVelocityProfiled(goalM_S);
    }

    private void dutyCycleAll() {
        m_servo1.setDutyCycle(1);
        m_servo2.setDutyCycle(1);
    }

    private void dutyCycleBackAll() {
        m_servo1.setDutyCycle(-1);
        m_servo2.setDutyCycle(-1);
    }
}
