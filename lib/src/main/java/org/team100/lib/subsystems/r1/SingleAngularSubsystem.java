package org.team100.lib.subsystems.r1;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.r.RDynamics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.servo.AngularPositionServo;
import org.team100.lib.servo.OutboardAngularPositionServo;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleAngularSubsystem extends SubsystemBase {
    private final LoggerFactory m_log1;
    private final AngularPositionServo m_servo1;

    public SingleAngularSubsystem(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId1,
            NeutralMode100 neutral,
            MotorPhase phase1,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid,
            double gearRatio,
            double initialPosition,
            RDynamics dynamics,
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
        RotaryMechanism lm1 = new RotaryMechanism(
                m_log1, m1, m1.encoder(), initialPosition, gearRatio,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

        m_servo1 = new OutboardAngularPositionServo(
                m_log1, lm1, dynamics, ref, xtolerance, vtolerance);
    }

    public Command voltage(double voltage) {
        return run(() -> setVoltage(voltage));
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

    public void close() {
        m_servo1.close();
    }

    private void reset() {
        m_servo1.reset();
    }

    private void setVoltage(double value) {
        m_servo1.setVoltage(value);
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
        m_servo1.setUnwrappedEncoderPositionRad(0);
    }

}
