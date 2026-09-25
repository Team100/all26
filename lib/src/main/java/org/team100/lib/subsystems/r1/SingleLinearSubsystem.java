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
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.servo.OutboardLinearPositionServo;
import org.team100.lib.util.CanId;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;

public class SingleLinearSubsystem extends SubsystemBase {
    private final LoggerFactory m_log1;
    private final LinearPositionServo m_servo1;

    public SingleLinearSubsystem(
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
            ReferenceR1 ref,
            double xtolerance,
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

        m_servo1 = new OutboardLinearPositionServo(
                m_log1, lm1, dynamics, ref, xtolerance, vtolerance);

    }

    public Command position(double position) {
        return startRun(this::reset,
                () -> setPositionProfiled(position));
    }

    public Command velocity(double velocity) {
        return startRun(this::reset,
                () -> setVelocity(velocity));
    }

    public Command stop() {
        return run(this::stopServo);
    }

    public Command zero() {
        return runOnce(this::setZero);
    }

    public boolean atGoal() {
        return m_servo1.atGoal();
    }

    @Override
    public void periodic() {
        m_servo1.periodic();
    }

    public void close() {
        m_servo1.close();
    }

    private void reset() {
        m_servo1.reset();
    }

    private void setPositionProfiled(double value) {
        m_servo1.setPositionProfiled(value);
    }

    private void setVelocity(double value) {
        m_servo1.setVelocity(value);
    }

    private void stopServo() {
        m_servo1.stop();
    }

    private void setZero() {
        m_servo1.setEncoderPositionM(0);
    }

}
