package org.team100.frc2026.subsystems;

import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.team100.frc2026.robot.CurrentLimits;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.dynamics.r.RDynamicsAnalytic;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter hood must be at the minimum position at startup.
 */
public class ShooterHood extends SubsystemBase {
    private static final double TUNING_SETTING = 0;
    private static final CanId CAN_ID = new CanId(13);
    // from Yotaro 3/12/26
    private static final double GEAR_RATIO = 270;
    private static final double MIN_POSITION_RAD = 0;
    // max extension is 0.5 3/12/26
    private static final double MAX_POSITION_RAD = 0.45;

    private final Supplier<OptionalDouble> m_angle;
    private final AngularPositionServo m_servo;

    /**
     * @param parent log
     * @param angle  angle for auto mode
     */
    public ShooterHood(LoggerFactory parent, TotalCurrentLog currentLog, Supplier<OptionalDouble> angle) {
        LoggerFactory log = parent.type(this);
        m_angle = angle;

        // mass is zero for now because dynamics gravity direction doesn't match.
        // TODO: make the coordinates here match.
        RDynamics dynamics = new RDynamicsAnalytic(0.000, 0.000, 0.007, 0.001);
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(8, 16, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);

        final BareMotor motor;
        switch (Identity.instance) {
            case TEST_BOARD_B0 -> {

                Friction friction = new Friction(0.350, 0.350, 0.0, 0.5);
                // tuned 3/12/26
                PIDConstants pid = PIDConstants.makePositionPID(1.0);

                motor = new NeoVortexCANSparkMotor(
                        log, currentLog, CAN_ID, NeutralMode100.COAST, MotorPhase.REVERSE,
                        CurrentLimits.SHOOTER_HOOD, friction, pid, 0, 0);

            }
            default -> {
                motor = new SimulatedBareMotor(log, 600);
            }
        }

        m_servo = OutboardAngularPositionServo.make(
                log, motor, dynamics, ref, GEAR_RATIO,
                MIN_POSITION_RAD, MIN_POSITION_RAD, MAX_POSITION_RAD);
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    /** Fixed angle for around 2.5m */
    public Command failsafe() {
        return setPosition(0.1)
                .withName("Hood failsafe");
    }

    /**
     * Use a profile to set the position according to the angle supplier.
     * Never ends, but stops the motor when interrupted.
     */
    public Command autoPosition() {
        return startRun(
                this::reset,
                this::autoPositionWork)
                .finallyDo(this::stopServo)
                .withName("Hood Auto Position");
    }

    /**
     * Use a profile to set the position to minimum.
     * Never ends.
     */
    public Command in() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(MIN_POSITION_RAD))
                .withName("Hood In");
    }

    /**
     * Use a profile to set the position to maximum.
     * Never ends.
     */
    public Command out() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(MAX_POSITION_RAD))
                .withName("Hood Out");
    }

    /**
     * Set the position to the tuning value in glass, without a profile.
     * Never ends.
     */
    public Command tune() {
        return startRun(
                this::reset,
                () -> actuateWithProfile(TUNING_SETTING))
                .withName("Tune Hood");
    }

    public Command stop() {
        return run(this::stopServo)
                .withName("Stop Hood");
    }

    public Command stopOnce() {
        return runOnce(this::stopServo)
                .withName("Stop Hood Once");
    }

    public boolean onTarget() {
        return m_servo.atGoal();
    }

    /** For testing friction only */
    public Command setVelocity(double x) {
        return startRun(
                this::reset,
                () -> {
                    m_servo.setVelocity(x);
                })
                .withName("set velocity");
    }

    public Command setPosition(double rad) {
        return startRun(
                this::reset,
                () -> {
                    m_servo.actuateWithProfile(rad);
                })
                .withName("set position");
    }

    /////////////////////////////////////////

    /** For testing. */
    double getUnwrappedPositionRad() {
        return m_servo.getUnwrappedPositionRad();
    }

    /** For testing. */
    ModelR1 getUnwrappedGoal() {
        return m_servo.getUnwrappedGoal();
    }

    private void reset() {
        m_servo.reset();
    }

    private void stopServo() {
        m_servo.stop();
    }

    /** Use a profile to set the position. */
    private void actuateWithProfile(double value) {
        m_servo.actuateWithProfile(value);
    }

    /** Do not use a profile. */
    @SuppressWarnings("unused")
    private void actuateDirect(double value) {
        m_servo.actuateDirect(value);
    }

    private void autoPositionWork() {
        m_angle.get().ifPresentOrElse(
                this::actuateWithProfile, this::stopServo);
    }

}
