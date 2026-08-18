package org.team100.frc2026.subsystems;

import java.util.OptionalDouble;
import java.util.function.Supplier;

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
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.CurrentLimitedExponentialVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final boolean DEBUG = false;
    private static final double TUNING_SETTING = 0;
    private static final double TEST_SPEED = 15;
    private static final CanId CAN_ID_1 = new CanId(2);
    private static final CanId CAN_ID_2 = new CanId(5);
    private static final CanId CAN_ID_3 = new CanId(14);
    private static final CanId CAN_ID_4 = new CanId(9);
    private static final double TOLERANCE_M_S = 1;
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIAMETER_M = .115;

    // private static final double FULL_SPEED = 30;

    private final Supplier<OptionalDouble> m_speed;

    private final OutboardLinearVelocityServo m_servo1;
    private final OutboardLinearVelocityServo m_servo2;
    private final OutboardLinearVelocityServo m_servo3;
    private final OutboardLinearVelocityServo m_servo4;

    /**
     * @param parent log
     * @param speed  speed (m/s) for auto mode
     */
    public Shooter(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            Supplier<OptionalDouble> speed) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("Shooter1");
        LoggerFactory log2 = log.name("Shooter2");
        LoggerFactory log3 = log.name("Shooter3");
        LoggerFactory log4 = log.name("Shooter4");
        m_speed = speed;

        // dynamics are actually about inertia, so we find the "effective"
        // dynamics here.
        PDynamics dynamics = PDynamics.drum(0.5, WHEEL_DIAMETER_M / 2);

        // tuned 3/12/26
        VelocityProfileR1 profile = new CurrentLimitedExponentialVelocityProfileR1(
                20, 20, 40, 60);
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, 1);
        final BareMotor m1;
        final BareMotor m2;
        final BareMotor m3;
        final BareMotor m4;
        switch (Identity.instance) {
            case TEST_BOARD_B0, COMP_BOT -> {

                // friction test 3/12/262
                Friction friction = new Friction(0.3, 0.25, 0.0, 0.5);
                // tuned 3/12/26
                PIDConstants pid = PIDConstants.makeVelocityPID(0.075);

                int averageDepth = 2;
                int measurementPeriod = 4;
                m1 = new NeoVortexCANSparkMotor(
                        log1, currentLog, CAN_ID_1, NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.SHOOTER, friction, pid, averageDepth, measurementPeriod);
                m2 = new NeoVortexCANSparkMotor(
                        log2, currentLog, CAN_ID_2, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.SHOOTER, friction, pid, averageDepth, measurementPeriod);
                m3 = new NeoVortexCANSparkMotor(
                        log3, currentLog, CAN_ID_3, NeutralMode100.COAST, MotorPhase.FORWARD,
                        CurrentLimits.SHOOTER, friction, pid, averageDepth, measurementPeriod);
                m4 = new NeoVortexCANSparkMotor(
                        log4, currentLog, CAN_ID_4, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.SHOOTER, friction, pid, averageDepth, measurementPeriod);

            }
            default -> {
                m1 = new SimulatedBareMotor(log1, 600);
                m2 = new SimulatedBareMotor(log2, 600);
                m3 = new SimulatedBareMotor(log3, 600);
                m4 = new SimulatedBareMotor(log4, 600);
            }
        }
        // note different gear ratio
        m_servo1 = OutboardLinearVelocityServo.make(
                log1, m1, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo2 = OutboardLinearVelocityServo.make(
                log2, m2, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo3 = OutboardLinearVelocityServo.make(
                log3, m3, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
        m_servo4 = OutboardLinearVelocityServo.make(
                log4, m4, dynamics, ref, GEAR_RATIO, WHEEL_DIAMETER_M, TOLERANCE_M_S);
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
        m_servo3.periodic();
        m_servo4.periodic();
    }

    public Command tune() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(TUNING_SETTING))
                .withName("Tune Shooter");
    }

    public Command shooterFullspeed() {
        return startRun(
                this::reset,
                // () -> setVelocityProfiled(FULL_SPEED))
                () -> dutyCycleAll())
                .withName("Shoot full speed");
    }

    /**
     * Use a profile to spin up the shooter wheels to the test speed.
     * Never ends.
     */
    public Command testRun() {
        return startRun(
                this::reset,
                () -> setVelocityProfiled(TEST_SPEED))
                .withName("Shooter Test");
    }

    public Command testShooterFullspeed() {
        return run(this::dutyCycleAll)
                .withName("Test shoot full speed");
    }

    public Command testMotor1Command() {
        return run(this::dutyCycle1)
                .withName("Motor 1 Spin");
    }

    public Command testMotor2Command() {
        return run(this::dutyCycle2)
                .withName("Motor 2 Spin");
    }

    public Command testMotor3Command() {
        return run(this::dutyCycle3)
                .withName("Motor 3 Spin");
    }

    public Command testMotor4Command() {
        return run(this::dutyCycle4)
                .withName("Motor 4 Spin");
    }

    /** Fixed speed for about 2.5m */
    public Command failsafe() {
        return setVelocity(14)
                .withName("Shooter failsafe");
    }

    /**
     * Run the drums at the supplied speed.
     * Never ends, but stops the motors when interrupted.
     */
    public Command auto() {
        return startRun(
                this::reset,
                this::autoWork)
                .finallyDo(this::stopMotor)
                .withName("Shoot Auto");
    }

    public Command stop() {
        return run(this::stopMotor)
                .withName("Stop Shooter");
    }

    public Command stopOnce() {
        return runOnce(this::stopMotor)
                .withName("Stop Shooter Once");
    }

    /**
     * Velocity error (m/s).
     * Positive error = too slow, negative error = too fast.
     * Note this is quite noisy.
     */
    public double meanError() {
        return (m_servo1.error() + m_servo2.error() + m_servo3.error()) / 4;
    }

    public Boolean atSpeed() {
        return (m_servo1.atGoal() && m_servo2.atGoal() && m_servo3.atGoal());
    }

    /** For testing friction only */
    public Command setVelocity(double meters_sec) {
        return startRun(
                this::reset,
                () -> {
                    m_servo1.setVelocityProfiled(meters_sec);
                    m_servo2.setVelocityProfiled(meters_sec);
                    m_servo3.setVelocityProfiled(meters_sec);
                    m_servo4.setVelocityProfiled(meters_sec);
                })
                .withName("set velocity");
    }

    /////////////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
        m_servo3.reset();
        m_servo4.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
        m_servo3.stop();
        m_servo4.stop();
    }

    @SuppressWarnings("unused")
    private void setVelocityDirect(double setpointM_S) {
        m_servo1.setVelocityDirect(setpointM_S);
        m_servo2.setVelocityDirect(setpointM_S);
        m_servo3.setVelocityDirect(setpointM_S);
        m_servo4.setVelocityDirect(setpointM_S);
    }

    private void setVelocityProfiled(double goalM_S) {
        m_servo1.setVelocityProfiled(goalM_S);
        m_servo2.setVelocityProfiled(goalM_S);
        m_servo3.setVelocityProfiled(goalM_S);
        m_servo4.setVelocityProfiled(goalM_S);
    }

    private void dutyCycleAll() {
        m_servo1.setDutyCycle(1);
        m_servo2.setDutyCycle(1);
        m_servo3.setDutyCycle(1);
        m_servo4.setDutyCycle(1);
    }

    private void dutyCycle1() {
        m_servo1.setDutyCycle(1);
    }

    private void dutyCycle2() {
        m_servo2.setDutyCycle(1);
    }

    private void dutyCycle3() {
        m_servo3.setDutyCycle(1);
    }

    private void dutyCycle4() {
        m_servo4.setDutyCycle(1);
    }

    /** Run the drums at the speed supplied */
    private void autoWork() {
        OptionalDouble speed = m_speed.get();
        if (speed.isPresent()) {
            if (DEBUG)
                System.out.printf("speed %f\n", speed.getAsDouble());
            setVelocityProfiled(speed.getAsDouble());
        } else {
            System.out.println("no speed for shoot auto");
            stopMotor();
        }

    }

}
