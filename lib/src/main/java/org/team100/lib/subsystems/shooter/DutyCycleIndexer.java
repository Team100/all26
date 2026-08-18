package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer using duty cycle control.
 */
public class DutyCycleIndexer extends SubsystemBase implements ShooterIndexer {
    private static final double BLIP_DURATION = 0.12;
    /** full output duty cycle */
    private final double m_full;
    private final BareMotor m_motor;

    public DutyCycleIndexer(LoggerFactory log, double full, BareMotor motor) {
        m_full = full;
        m_motor = motor;
    }

    @Override
    public Command single() {
        return run(this::full)
                .withTimeout(BLIP_DURATION);
    }

    @Override
    public Command continuous() {
        return run(this::full);
    }

    @Override
    public Command stop() {
        return run(this::zero);
    }

    @Override
    public void periodic() {
        m_motor.periodic();
    }

    ////////////////////////////////////////////////////

    private void full() {
        m_motor.setDutyCycle(m_full);
    }

    private void zero() {
        m_motor.stop();
    }
}
