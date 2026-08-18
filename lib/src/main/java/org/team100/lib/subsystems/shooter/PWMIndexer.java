package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer using continuous-rotation servo or PWM controller.
 */
public class PWMIndexer extends SubsystemBase implements ShooterIndexer {
    private final PWM m_pwm;
    private final DoubleLogger m_log_dutyCycle;

    public PWMIndexer(LoggerFactory parent, RoboRioChannel channel) {
        LoggerFactory logger = parent.type(this);
        m_log_dutyCycle = logger.doubleLogger(Level.TRACE, "duty cycle");
        m_pwm = new PWM(channel.channel);
    }

    @Override
    public Command single() {
        return run(this::full)
                .withTimeout(0.5);
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
        //
    }

    //////////////////////////////////////////////////////////

    private void full() {
        set(1);
    }

    private void zero() {
        set(0);
    }

    private void set(double dutyCycle) {
        m_pwm.setSpeed(dutyCycle);
        m_log_dutyCycle.log(() -> dutyCycle);
    }
}
