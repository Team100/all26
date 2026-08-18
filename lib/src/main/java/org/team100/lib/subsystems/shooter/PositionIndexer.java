package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r1.SetpointsR1;
import org.team100.lib.servo.LinearPositionServo;
import org.team100.lib.state.ControlR1;

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

    @Override
    public void periodic() {
        m_servo.periodic();
    }

    //////////////////////////////////////////////////////////

    private void stepGoal() {
        m_goal += 0.2;
    }

    private void goToGoal() {
        if (m_profiled) {
            m_servo.setPositionProfiled(m_goal);
        } else {
            ControlR1 c = new ControlR1(m_goal);
            m_servo.setPositionDirect(new SetpointsR1(c, c));
        }
    }

    private void zero() {
        m_servo.stop();
    }

}
