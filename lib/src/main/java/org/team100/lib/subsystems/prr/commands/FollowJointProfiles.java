package org.team100.lib.subsystems.prr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.prr.SubsystemPRR;

import edu.wpi.first.math.MathUtil;

/**
 * Follow three uncoordinated profiles in configuration space.
 * Starting point and velocity are current measurements.
 */
public class FollowJointProfiles extends MoveAndHold {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final SubsystemPRR m_subsystem;
    private final ModelR1 m_g1;
    private final ModelR1 m_g2;
    private final ModelR1 m_g3;
    private final ProfileR1 m_p1;
    private final ProfileR1 m_p2;
    private final ProfileR1 m_p3;

    private ControlR1 m_c1;
    private ControlR1 m_c2;
    private ControlR1 m_c3;

    public FollowJointProfiles(
            SubsystemPRR subsystem,
            PRRConfig goal,
            ProfileR1 p1,
            ProfileR1 p2,
            ProfileR1 p3) {
        m_subsystem = subsystem;
        // Joint goals are motionless
        m_g1 = new ModelR1(goal.q1(), 0);
        m_g2 = new ModelR1(goal.q2(), 0);
        m_g3 = new ModelR1(goal.q3(), 0);
        m_p1 = p1;
        m_p2 = p2;
        m_p3 = p3;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // initial position is current position
        PRRConfig c = m_subsystem.getConfig();
        // initial velocity is current velocity
        PRRVelocity jv = m_subsystem.getJointVelocity();
        m_c1 = new ControlR1(c.q1(), jv.q1dot());
        m_c2 = new ControlR1(c.q2(), jv.q2dot());
        m_c3 = new ControlR1(c.q3(), jv.q3dot());
    }

    @Override
    public void execute() {
        m_c1 = m_p1.calculate(DT, m_c1, m_g1);
        m_c2 = m_p2.calculate(DT, m_c2, m_g2);
        m_c3 = m_p3.calculate(DT, m_c3, m_g3);
        PRRConfig c = new PRRConfig(m_c1.x(), m_c2.x(), m_c3.x());
        PRRVelocity jv = new PRRVelocity(m_c1.v(), m_c2.v(), m_c3.v());
        PRRAcceleration ja = new PRRAcceleration(m_c1.v(), m_c2.v(), m_c3.v());
        m_subsystem.set(c, jv, ja);
    }

    @Override
    public boolean isDone() {
        return profileDone() && atReference();
    }

    @Override
    public double toGo() {
        return 0;
    }

    /** The profile has reached the goal. */
    private boolean profileDone() {
        return MathUtil.isNear(m_g1.x(), m_c1.x(), 0.01)
                && MathUtil.isNear(m_g2.x(), m_c2.x(), 0.02)
                && MathUtil.isNear(m_g3.x(), m_c3.x(), 0.02)
                && Math.abs(m_c1.v()) < 0.01
                && Math.abs(m_c2.v()) < 0.02
                && Math.abs(m_c3.v()) < 0.02;

    }

    /** The measurement has reached the goal. */
    private boolean atReference() {
        PRRConfig c = m_subsystem.getConfig();
        PRRVelocity jv = m_subsystem.getJointVelocity();
        return MathUtil.isNear(m_g1.x(), c.q1(), 0.01)
                && MathUtil.isNear(m_g2.x(), c.q2(), 0.02)
                && MathUtil.isNear(m_g3.x(), c.q3(), 0.02)
                && Math.abs(jv.q1dot()) < 0.01
                && Math.abs(jv.q2dot()) < 0.02
                && Math.abs(jv.q3dot()) < 0.02;
    }

}
