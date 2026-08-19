package org.team100.frc2026.subsystems;

import org.team100.lib.config.CurrentLimit;
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
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class Climber extends SubsystemBase {
    public static final double GEAR_RATIO = 28;
    private static final double L0 = 0;
    private static final double L1 = -Math.PI / 2;
    private static final double L3 = -Math.PI;

    private final AngularPositionServo m_servo1;
    private final AngularPositionServo m_servo2;

    public Climber(LoggerFactory parent, TotalCurrentLog currentLog) {
        LoggerFactory log = parent.type(this);
        LoggerFactory log1 = log.name("motor1");
        LoggerFactory log2 = log.name("motor2");
        // Dynamics are unimportant for this mechanism.
        RDynamics dynamics = new RDynamicsAnalytic(0, 0, 0, 0);
        ProfileR1 profile = new TrapezoidProfileR1(3, 5, 0.05);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, 0.05, 0.05);
        double initialPosition = 0;
        final BareMotor m1;
        final BareMotor m2;
        switch (Identity.instance) {
            case TEST_BOARD_B0 -> {
                CurrentLimit limit = new CurrentLimit(60, 40);
                Friction friction = new Friction(0, 0, 0, 0);
                PIDConstants pid = new PIDConstants(1, 0, 0, 0, 0, 0);
                m1 = new KrakenX60Motor(
                        log1, currentLog, new CanId(6),
                        NeutralMode100.BRAKE, MotorPhase.FORWARD,
                        limit,
                        friction, pid);
                m2 = new KrakenX60Motor(
                        log2, currentLog, new CanId(7),
                        NeutralMode100.BRAKE, MotorPhase.FORWARD,
                        limit,
                        friction, pid);
            }
            default -> {
                m1 = new SimulatedBareMotor(log1, 600);
                m2 = new SimulatedBareMotor(log2, 600);
            }
        }
        m_servo1 = OutboardAngularPositionServo.make(
                log1, m1, dynamics, ref, GEAR_RATIO, initialPosition, 0, 100);
        m_servo2 = OutboardAngularPositionServo.make(
                log2, m2, dynamics, ref, GEAR_RATIO, initialPosition, 0, 100);
    }

    public Command setClimb0() {
        return startRun(this::reset, () -> actuateWithProfile(L0)).withName("Climb L0");
    }

    public Command setClimb1() {
        return startRun(this::reset, () -> actuateWithProfile(L1)).withName("Climb L1");
    }

    public Command setClimb3() {
        return startRun(this::reset, () -> actuateWithProfile(L3)).withName("Climb L3");
    }

    public Command stop() {
        return run(this::stopMotor).withName("Stop Climber");
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
        m_servo2.periodic();
    }

    ///////////////////////////////////////

    private void reset() {
        m_servo1.reset();
        m_servo2.reset();
    }

    private void stopMotor() {
        m_servo1.stop();
        m_servo2.stop();
    }

    private void actuateWithProfile(double unwrappedGoalRad) {
        m_servo1.actuateWithProfile(unwrappedGoalRad);
        m_servo2.actuateWithProfile(unwrappedGoalRad);
    }
}
