package org.team100.lib.subsystems.rrr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;
import org.team100.lib.subsystems.rrr.RRRArm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

/**
 * Use a 1d profile to coordinate all axes to the goal config.
 */
public class MoveToConfigWithProfile extends MoveAndHold {
    private final RRRArm m_arm;
    private final RRRConfig m_goal;
    /** Profile walks from 0 to 1. */
    private final ProfileR1 m_profile;

    // this is the config at initialize
    private RRRConfig m_start;

    // unit vector (using the distance metric) pointing from start to goal.
    private Vector<N3> m_unit;

    private ControlR1 m_setpoint;
    private StateR1 m_profileGoal;

    public MoveToConfigWithProfile(RRRArm arm, ProfileR1 profile, RRRConfig goal) {
        m_arm = arm;
        m_goal = goal;
        m_profile = profile;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getConfig();
        double distance = m_start.euclideanDistance(m_goal);
        m_unit = RRRConfig.unit(m_start, m_goal);
        // assumes initial velocity is zero :(
        m_setpoint = new ControlR1();
        // scale the profile to the norm
        m_profileGoal = new StateR1(distance, 0);
    }

    @Override
    public void execute() {
        if (isDone()) {
            return;
        }
        m_setpoint = m_profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                m_setpoint,
                m_profileGoal);
        RRRConfig q = m_start.plus(RRRConfig.fromVector(m_unit.times(m_setpoint.x())));
        RRRVelocity qdot = RRRVelocity.fromVector(m_unit.times(m_setpoint.v()));
        RRRAcceleration qddot = RRRAcceleration.fromVector(m_unit.times(m_setpoint.a()));
        m_arm.set(q, qdot, qddot);
    }

    @Override
    public boolean isDone() {
        return toGo() < 0.01;
    }

    @Override
    public double toGo() {
        return m_arm.getConfig().euclideanDistance(m_goal);
    }
}
