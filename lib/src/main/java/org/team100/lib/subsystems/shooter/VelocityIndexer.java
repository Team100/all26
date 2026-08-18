package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer using velocity control.
 */
public class VelocityIndexer extends SubsystemBase implements ShooterIndexer {
    private static final double SINGLE_DURATION = 0.1;
    /** full output velocity */
    private final double m_fullVelocityM_S;
    private final LinearVelocityServo m_servo;
    private final boolean m_profiled;

    public VelocityIndexer(
            LoggerFactory log,
            double fullVelocityM_S,
            LinearVelocityServo servo,
            boolean profiled) {
        m_fullVelocityM_S = fullVelocityM_S;
        m_servo = servo;
        m_profiled = profiled;
    }

    @Override
    public Command single() {
        return run(this::full)
                .withTimeout(SINGLE_DURATION);
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
        m_servo.periodic();
    }

    //////////////////////////////////////////////////////////

    private void full() {
        set(m_fullVelocityM_S);
    }

    private void zero() {
        m_servo.stop();
    }

    private void set(double velocityM_S) {
        if (m_profiled) {
            m_servo.setVelocityProfiled(velocityM_S);
        } else {
            m_servo.setVelocityDirect(velocityM_S);
        }
    }

}
