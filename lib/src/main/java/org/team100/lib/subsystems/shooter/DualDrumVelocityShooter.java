package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter with left and right drums.
 */
public class DualDrumVelocityShooter extends SubsystemBase implements DualDrumShooter {
    /** full output velocity */
    private final double m_full;
    private final LinearVelocityServo m_left;
    private final LinearVelocityServo m_right;
    private final boolean m_profiled;
    private final BooleanLogger m_log_atGoal;

    public DualDrumVelocityShooter(
            LoggerFactory log,
            double full,
            LinearVelocityServo left,
            LinearVelocityServo right,
            boolean profiled) {
        m_full = full;
        m_left = left;
        m_right = right;
        m_profiled = profiled;
        m_log_atGoal = log.booleanLogger(Level.TRACE, "At goal");
    }

    @Override
    public Command spinSlow() {
        return run(this::half);
    }

    @Override
    public Command spinFast() {
        return run(this::full);
    }

    @Override
    public Command stop() {
        return run(this::zero);
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
        m_log_atGoal.log(this::atGoal);
    }

    public double getVelocity() {
        return (m_left.getVelocity() + m_right.getVelocity()) / 2;
    }

    public boolean atGoal() {
        return m_right.atGoal() && m_left.atGoal();
    }
    ///////////////////////////////////////////////////////

    private void half() {
        set(m_full / 2);
    }

    private void full() {
        set(m_full);
    }

    private void set(double x) {
        if (m_profiled) {
            m_left.setVelocityProfiled(x);
            m_right.setVelocityProfiled(x);
        } else {
            m_left.setVelocityDirect(x);
            m_right.setVelocityDirect(x);
        }
    }

    private void zero() {
        m_left.stop();
        m_right.stop();
    }
}