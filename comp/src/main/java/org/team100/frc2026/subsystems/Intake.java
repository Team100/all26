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

public class Intake extends SubsystemBase {
    private static final CanId CAN_ID_1 = new CanId(20);
    private static final CanId CAN_ID_2 = new CanId(16);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 2;
    private static final double WHEEL_DIAMETER_M = 0.05;
    private static final double NORMAL_SPEED = 10;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    public Intake(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("motor1");
        LoggerFactory log2 = log.name("motor2");
        // equivalent linear dynamics for the actual drum inertia.
        PDynamics dynamics = PDynamics.drum(0.001, 0.025);
        // VelocityProfileR1 profile = new CurrentLimitedExponentialVelocityProfileR1(
        // 10, 10, 20, 30);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(
                20, 50);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {

                // friction test 3/12/26
                Friction friction = new Friction(0.5, 0.5, 0.0, 0.5);
                // tuned 3/12/26
                PIDConstants pid = PIDConstants.makeVelocityPID(0.08);
                m1 = new KrakenX44Motor(
                        log1, currentLog, CAN_ID_1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.INTAKE, friction, pid);
                m2 = new KrakenX44Motor(
                        log2, currentLog, CAN_ID_2, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.INTAKE, friction, pid);
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

    /**
     * Use a profile to spin up the roller to the normal speed.
     * Never ends, but stops the motor when interrupted.
     */
    public Command intake() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(NORMAL_SPEED))
                .finallyDo(this::stopMotor)
                .withName("Intake Normal Speed");
    }

    /**
     * Roll backwards to clear jams.
     */
    public Command back() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(-5))
                .withName("Intake back");
    }

    /** Stop forever */
    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Intake");
    }

    /** Stop and then end */
    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Intake Once");
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
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

    ////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void setVelocityProfiled(double velocityM_S) {
        m_servo1.setVelocityProfiled(velocityM_S);
        m_servo2.setVelocityProfiled(velocityM_S);
    }
}
