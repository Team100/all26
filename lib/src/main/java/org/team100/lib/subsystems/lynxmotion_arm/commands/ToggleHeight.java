package org.team100.lib.subsystems.lynxmotion_arm.commands;

import org.team100.lib.subsystems.lynxmotion_arm.LynxArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** If we're at zero, move to safe. If we're not at zero, move there. */
public class ToggleHeight extends Command {
    private static final double DURATION = 1.0;
    private final LynxArm m_arm;
    private final Timer m_timer;

    private Pose3d m_startPose;
    private double m_startGrip;
    private double m_goal;

    public ToggleHeight(LynxArm arm) {
        m_arm = arm;
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_startPose = m_arm.getPosition().p6();
        m_startGrip = m_arm.getGrip();
        if (m_startPose.getZ() < 0.01) {
            // it's down, we want to go up.
            m_goal = 0.05;
        } else {
            // it's up, we want to go down.
            m_goal = 0;
        }
        m_timer.restart();
    }

    @Override
    public void execute() {
        double s = m_timer.get() / DURATION;
        double z = MathUtil.interpolate(m_startPose.getZ(), m_goal, s);
        Pose3d newPose = new Pose3d(
                new Translation3d(m_startPose.getX(), m_startPose.getY(), z),
                m_startPose.getRotation());
        m_arm.setPosition(newPose);
        // the servos hold position without being commanded but it's still a good habit
        // to always command all the axes of whatever subsystem the command requires.
        m_arm.setGrip(m_startGrip);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(DURATION);
    }

}
