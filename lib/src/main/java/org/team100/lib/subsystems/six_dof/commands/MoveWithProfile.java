package org.team100.lib.subsystems.six_dof.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.six_dof.SixDofArm;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Move the arm to the goal, endlessly.
 * 
 * Uses a single profile.
 * 
 * Interpolates each joint separately, so this is
 * immune to singularities (except at the endpoints),
 * but it might exceed the workspace limits, e.g.
 * by hitting the floor.
 */
public class MoveWithProfile extends MoveAndHold {

    private final SixDofArm m_arm;
    private final Pose3d m_goal;
    /** Profile walks from 0 to 1. */
    private final ProfileR1 m_profile;

    // this is the config at initialize
    private SixDofConfig m_start;
    // config goal depends on start, to choose the closest one.
    private SixDofConfig m_configGoal;

    private ControlR1 m_setpoint;
    // for now this is always 1.
    private ModelR1 m_profileGoal;

    /**
     * @param arm
     * @param goal tool points at +x
     */
    public MoveWithProfile(SixDofArm arm, Pose3d goal) {
        m_arm = arm;
        m_goal = goal;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");
        m_profile = new WPITrapezoidProfileR1(1, 1);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getConfig();
        m_configGoal = m_arm.config(m_goal);
        if (m_configGoal == null)
            throw new IllegalArgumentException(
                    "infeasible goal: " + StrUtil.poseStr(m_goal));
        m_setpoint = new ControlR1();
        m_profileGoal = new ModelR1(1, 0);
    }

    @Override
    public void execute() {
        m_setpoint = m_profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                m_setpoint,
                m_profileGoal);

        double s = m_setpoint.x();

        SixDofConfig c = SixDofConfig.interpolate(m_start, m_configGoal, s);
        m_arm.setConfig(c);
    }

    @Override
    public boolean isDone() {
        return toGo() < 0.01;
    }

    @Override
    public double toGo() {
        return m_arm.getConfig().distance(m_configGoal);
    }

}
