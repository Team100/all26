package org.team100.frc2025.Climber;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX60Motor;
import org.team100.lib.motor.sim.LazySimulatedBareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIntake extends SubsystemBase {

    private final BareMotor m_motor;
    private int count;

    public ClimberIntake(LoggerFactory parent, TotalCurrentLog currentLog, CanId canID) {
        LoggerFactory log = parent.type(this);
        count = 0;
        switch (Identity.instance) {
            case COMP_BOT -> {
                m_motor = new KrakenX60Motor(
                        log, currentLog,
                        canID, NeutralMode100.COAST, MotorPhase.REVERSE,
                        new CurrentLimit(20, 20),
                        new Friction(0.26, 0.26, 0.006, 0.5),
                        PIDConstants.zero());
            }
            default -> {
                m_motor = new LazySimulatedBareMotor(
                        log, new SimulatedBareMotor(log, 600), 1.5);
            }
        }
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    public boolean isSlow() {
        return m_motor.getVelocityRad_S() < 1;
    }

    public boolean intaking() {
        return count > 0;
    }

    public boolean isIn() {
        return count > 25;
    }

    // COMMANDS

    public Command stop() {
        return run(this::stopMotor);
    }

    public Command intake() {
        return startRun(
                () -> count = 0,
                () -> {
                    fullSpeed();
                    if (isSlow()) {
                        count++;
                    }
                });
    }

    ////////////////

    public void stopMotor() {
        m_motor.setDutyCycle(0);
    }

    private void fullSpeed() {
        m_motor.setDutyCycle(1);
    }

}
