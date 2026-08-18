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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public static final CanId canID1 = new CanId(8);
    public static final CanId canID2 = new CanId(9);
    private static final double TOLERANCE_M_S = 1.0;
    private static final double GEAR_RATIO = 3.0;
    private static final double WHEEL_DIAMETER_M = 0.05;
    // TODO: TUNE
    private static final double NORMAL_SPEED = 10.0;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;

    private final Shooter m_Shooter;

    public Feeder(LoggerFactory parent, TotalCurrentLog currentLog, Shooter shooter) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Feeder1");
        LoggerFactory log2 = log.name("Feeder2");
        m_Shooter = shooter;
        // equivalent linear dynamics for the actual drum inertia.
        PDynamics dynamics = PDynamics.drum(0.001, 0.025);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(40);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        switch (Identity.instance) {
            case TEST_BOARD_B0 -> {
                // friction test 3/12/26
                Friction friction = new Friction(0.9, 0.9, 0.0, 0.5);
                // tuned 3/12/26
                PIDConstants pid = PIDConstants.makeVelocityPID(0.05);

                m1 = new KrakenX44Motor(
                        log1, currentLog, canID1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.FEEDER, friction, pid);
                m2 = new KrakenX44Motor(
                        log2, currentLog, canID2, NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.FEEDER, friction, pid);
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

    /** Feed 100% when shooter is at speed, 0% otherwise */
    public Command bangbang() {
        return startRun(
                this::reset,
                this::feedWhenReady)
                .withName("Feed");
    }

    /**
     * Use a profile to spin up the feeder to the normal speed.
     * Never ends.
     */
    public Command normal() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(NORMAL_SPEED))
                .withName("Feed Normally");
    }

    /**
     * Feed rate slowdown is proportional to shooter speed error outside the
     * tolerance. Never ends, but stops the motor when interrupted.
     */
    public Command proportional() {
        return startRun(
                this::reset,
                () -> shootProportional())
                .finallyDo(this::stopMotor)
                .withName("Feed Proportional");
    }

    public Command back() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(-5))
                .withName("Feed back");
    }

    public Command testFeed() {
        return run(() -> setDutyCycle(1))
                .withName("Test Feed");
    }

    public Command testFeedBack() {
        return run(() -> setDutyCycle(-1))
                .withName("Test Feed Back");
    }

    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Feeder");
    }

    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Feeder Once");
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

    ///////////////////////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void shootProportional() {
        // Error (m/s), positive = too slow.
        // Note this is quite noisy
        double meanError = m_Shooter.meanError();
        // Only care about slowness (positive)
        double shooterSlowness = Math.max(0, meanError);
        // if we're 1 m/s slow, feed at 100%
        // if we're 2 m/s slow, feed at 0%
        double feedFraction = MathUtil.clamp(2.0 - shooterSlowness, 0, 1);
        double feedSpeed = NORMAL_SPEED * feedFraction;
        setVelocityProfiled(feedSpeed);
    }

    private void feedWhenReady() {
        if (m_Shooter.atSpeed()) {
            setVelocityProfiled(NORMAL_SPEED);
        } else {
            setVelocityProfiled(0);
        }
    }

    private void setVelocityProfiled(double x) {
        m_servo1.setVelocityProfiled(x);
        m_servo2.setVelocityProfiled(x);
    }

    private void setDutyCycle(double x) {
        m_servo1.setDutyCycle(x);
        m_servo2.setDutyCycle(x);
    }
}
