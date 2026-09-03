package org.team100.lib.subsystems.six_dof.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;
import org.team100.lib.subsystems.six_dof.SixDofArm;
import org.team100.lib.util.StrUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;

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
    private static final boolean DEBUG = false;
    private final SixDofArm m_arm;
    private final Pose3d m_goal;
    /** Profile walks from 0 to 1. */
    private final ProfileR1 m_profile;

    // this is the config at initialize
    private SixDofConfig m_start;
    // config goal depends on start, to choose the closest one.
    private SixDofConfig m_configGoal;
    // unit vector (using the distance metric) pointing from start to goal.
    private Vector<N6> m_unit;

    private ControlR1 m_setpoint;
    // for now this is always 1.
    private StateR1 m_profileGoal;

    /**
     * @param arm
     * @param goal tool points at +x
     */
    public MoveWithProfile(SixDofArm arm, ProfileR1 profile, Pose3d goal) {
        m_arm = arm;
        m_goal = goal;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");
        m_profile = profile;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getConfig();
        m_configGoal = m_arm.config(m_goal);
        double distance = m_start.distance(m_configGoal);
        m_unit = SixDofConfig.unit(m_start, m_configGoal);

        if (m_configGoal == null)
            throw new IllegalArgumentException(
                    "infeasible goal: " + StrUtil.poseStr(m_goal));
        m_setpoint = new ControlR1();
        // the profile length is the distance
        m_profileGoal = new StateR1(distance, 0);
    }

    @Override
    public void execute() {
        double distance = m_profileGoal.x();
        if (distance < 1e-6) {
            // we're already there
            return;
        }
        m_setpoint = m_profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                m_setpoint,
                m_profileGoal);

        if (DEBUG)
            System.out.printf("MoveWithProfile.execute() setpoint: %s\n", m_setpoint);

        SixDofConfig q = m_start.plus(SixDofConfig.fromVector(m_unit.times(m_setpoint.x())));
        SixDofVelocity qdot = SixDofVelocity.fromVector(m_unit.times(m_setpoint.v()));
        SixDofAcceleration qddot = SixDofAcceleration.fromVector(m_unit.times(m_setpoint.a()));
        if (DEBUG)
            System.out.printf("MoveWithProfile.execute() q: %s\n", q);

        m_arm.set(q, qdot, qddot);
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
