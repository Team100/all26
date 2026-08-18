package org.team100.frc2026.subsystems;

import org.team100.lib.config.CurrentLimit;
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
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberExtension extends SubsystemBase {
    public static final double WHEEL_DIAMETER_M = 0.001275;
    public static final double GEAR_RATIO = 1;
    private static final double MAX_EXTENSION_M = 0.25;
    private static final double MIN_EXTENSION_M = 0.01;

    private final LinearPositionServo m_servo;

    public ClimberExtension(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        ProfileR1 profile = new TrapezoidProfileR1(0.1, 2, 0.05);
        // dynamics are unimportant for this subsystem.
        PDynamics dyn = new PDynamics(0);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        final BareMotor motor;
        switch (Identity.instance) {
            case TEST_BOARD_6B -> {
                CurrentLimit limit = new CurrentLimit(40, 40);
                Friction friction = new Friction(0, 0, 0, 0);
                PIDConstants pid = new PIDConstants(1, 0, 0, 0, 0, 0);
                motor = new NeoVortexCANSparkMotor(
                        log, currentLog, new CanId(2),
                        NeutralMode100.BRAKE, MotorPhase.FORWARD,
                        limit, friction, pid, 0, 0);
            }
            default -> {
                motor = new SimulatedBareMotor(log, 600);
            }
        }
        m_servo = OutboardLinearPositionServo.make(
                log, motor, dyn, ref, GEAR_RATIO, WHEEL_DIAMETER_M);
    }

    public Command setPosition() {
        return startRun(this::reset, () -> setPositionProfiled(MAX_EXTENSION_M)).withName("Climber Extension Extend");
    }

    public Command setHomePosition() {
        return startRun(this::reset, () -> setPositionProfiled(MIN_EXTENSION_M)).withName("Climber Extension Retract");
    }

    public Command stop() {
        return run(this::stopMotor).withName("Stop Climber Extension");
    }

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    ///////////////////////////////////////////

    private void reset() {
        m_servo.reset();
    }

    private void stopMotor() {
        m_servo.stop();
    }

    private void setPositionProfiled(double goalM) {
        m_servo.setPositionProfiled(goalM);
    }
}
