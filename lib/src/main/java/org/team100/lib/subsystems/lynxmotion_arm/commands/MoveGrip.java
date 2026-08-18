package org.team100.lib.subsystems.lynxmotion_arm.commands;

import org.team100.lib.subsystems.lynxmotion_arm.LynxArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveGrip extends Command {
    private static final double DURATION = 0.5;
    private final LynxArm m_arm;
    private final double m_goal;
    private final Timer m_timer;

    private Pose3d m_startPose;
    private double m_startGrip;

    public MoveGrip(LynxArm arm, double goal) {
        m_arm = arm;
        m_goal = goal;
        m_timer = new Timer();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_startPose = m_arm.getPosition().p6();
        m_startGrip = m_arm.getGrip();
        m_timer.restart();
    }

    @Override
    public void execute() {
        double s = m_timer.get() / DURATION;
        double grip = MathUtil.interpolate(m_startGrip, m_goal, s);
        m_arm.setGrip(grip);
        m_arm.setPosition(m_startPose);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(DURATION);
    }

}
