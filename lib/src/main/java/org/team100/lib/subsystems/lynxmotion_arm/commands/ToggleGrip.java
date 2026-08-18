package org.team100.lib.subsystems.lynxmotion_arm.commands;

import org.team100.lib.subsystems.lynxmotion_arm.LynxArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * If the grip is open, close it. If it's closed, open it.
 * 
 * Since this is a separate command from any end-effector-moving commands, it
 * will obviously briefly stop the motion when it runs. The expectation is that
 * this will run when you're stopped anyway (to pick or place).
 */
public class ToggleGrip extends Command {
    private static final double CLOSED = 0.001;
    private static final double OPEN = 0.0325;
    private static final double DURATION = 0.5;
    private final LynxArm m_arm;
    private final Timer m_timer;

    private Pose3d m_startPose;
    private double m_startGrip;
    private double m_goal;

    public ToggleGrip(LynxArm arm) {
        m_arm = arm;
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_startPose = m_arm.getPosition().p6();
        m_startGrip = m_arm.getGrip();
        if (m_startGrip < 0.01) {
            // it's closed, we want to open it.
            m_goal = OPEN;
        } else {
            // it's open, we want to close it.
            m_goal = CLOSED;
        }
        m_timer.restart();
    }

    @Override
    public void execute() {
        double s = m_timer.get() / DURATION;
        double grip = MathUtil.interpolate(m_startGrip, m_goal, s);
        m_arm.setGrip(grip);
        // the servos hold position without being commanded but it's still a good habit
        // to always command all the axes of whatever subsystem the command requires.
        m_arm.setPosition(m_startPose);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(DURATION);
    }

}
