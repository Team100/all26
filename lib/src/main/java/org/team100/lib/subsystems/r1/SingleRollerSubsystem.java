package org.team100.lib.subsystems.r1;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;

public class SingleRollerSubsystem extends SubsystemBase {
    private final LoggerFactory m_log1;
    private final LinearVelocityServo m_servo1;

    public SingleRollerSubsystem(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId1,
            NeutralMode100 neutral,
            MotorPhase phase1,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid,
            double gearRatio,
            double wheelDiameterM,
            PDynamics dynamics,
            VelocityReferenceR1 ref,
            double vtolerance,
            boolean enable) {
        LoggerFactory log = parent.type(this);
        m_log1 = log.name("1");
        final Motor m1;
        if (enable && RobotBase.isReal()) {
            m1 = new KrakenX44Motor(
                    m_log1, currentLog, canId1, neutral, phase1, limit, friction, pid);
        } else {
            m1 = new SimulatedMotor(m_log1, 600);
        }
        LinearMechanism lm1 = new LinearMechanism(
                m_log1, m1, m1.encoder(), gearRatio, wheelDiameterM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        m_servo1 = new OutboardLinearVelocityServo(
                m_log1, lm1, dynamics, ref, vtolerance);
    }

    public Command voltage(double voltage) {
        return run(() -> setVoltage(voltage));
    }

    public Command velocity(double velocity) {
        return startRun(this::reset,
                () -> setVelocity(velocity));
    }

    public Command stop() {
        return run(this::stopServo);
    }

    public boolean atGoal() {
        return m_servo1.atGoal();
    }

    public void close() {
        m_servo1.close();
    }

    private void reset() {
        m_servo1.reset();
    }

    private void setVoltage(double value) {
        m_servo1.setVoltage(value);
    }

    private void setVelocity(double value) {
        m_servo1.setVelocityProfiled(value);
    }

    private void stopServo() {
        m_servo1.stop();
    }
}
