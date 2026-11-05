package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.servo.LinearPositionServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer using position control.
 */
public class PositionIndexer extends SubsystemBase implements ShooterIndexer {
    private final LinearPositionServo m_servo;
    private final boolean m_profiled;
    private double m_goal;

    public PositionIndexer(LoggerFactory log, LinearPositionServo servo, boolean profiled) {
        m_servo = servo;
        m_profiled = profiled;
        m_goal = m_servo.getPosition();
    }

    @Override
    public Command single() {
        // Advance the goal one ball-width and then go until there.
        return startRun(
                this::stepGoal,
                this::goToGoal)
                .until(m_servo::atGoal);
    }

    @Override
    public Command continuous() {
        return single().withTimeout(0.1).repeatedly();
    }

    @Override
    public Command stop() {
        return run(this::zero);
    }

    //////////////////////////////////////////////////////////

    private void stepGoal() {
        m_goal += 0.2;
    }

    private void goToGoal() {
        if (m_profiled) {
            m_servo.setPositionProfiled(m_goal);
        } else {
            m_servo.setPositionDirect(m_goal);
        }
    }

    private void zero() {
        m_servo.stop();
    }

}
