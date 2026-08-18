package org.team100.lib.subsystems.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive shooter with a single drum.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class SingleDrumShooter extends SubsystemBase {
    private static final double TARGET_VELOCITY_M_S = 10;

    private final LinearVelocityServo m_drum;
    private final BooleanLogger m_log_atGoal;

    public SingleDrumShooter(
            LoggerFactory log,
            LinearVelocityServo drum) {
        m_drum = drum;
        m_log_atGoal = log.booleanLogger(Level.TRACE, "At goal");
    }

    public void set(double velocityM_S) {
        m_drum.setVelocityProfiled(velocityM_S);
    }

    public void stop() {
        set(0);
    }

    public void spinUp() {
        set(TARGET_VELOCITY_M_S);
    }

    public void spinUp(double x) {
        set(x);
    }

    public boolean atGoal() {
        return m_drum.atGoal();
    }

    public Command spin() {
        return run(this::spinUp);
    }

    @Override
    public void periodic() {
        m_drum.periodic();
        m_log_atGoal.log(this::atGoal);
    }
}