package org.team100.lib.subsystems.rrr.commands;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.hid.DriverVelocity;
import org.team100.lib.subsystems.rrr.RRRArm;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Control joint velocity directly.
 */
public class MoveJointsManually extends Command {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final RRRArm m_arm;
    private final Supplier<DriverVelocity> m_v;
    private RRRConfig m_q;
    private RRRVelocity m_qdot = new RRRVelocity(0, 0, 0);

    public MoveJointsManually(RRRArm arm, Supplier<DriverVelocity> v) {
        m_arm = arm;
        m_v = v;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_q = m_arm.getConfig();
    }

    @Override
    public void execute() {
        // controller specifies velocity in range [-1, 1]
        DriverVelocity v = m_v.get();
        // This is a sort of arbitrary mapping of "velocity"
        // fields to joints.
        double q1dot = -15.0 * v.theta(); // axis 0
        double q2dot = -5.0 * v.y(); // axis 4
        double q3dot = -5.0 * v.x(); // axis 5
        // Desired velocity in rad/s.
        RRRVelocity qdot = new RRRVelocity(q1dot, q2dot, q3dot);
        // Desired accel in rad/s/s.
        RRRAcceleration qddot = qdot.accel(m_qdot, DT);
        // Desired position in rad.
        RRRConfig q = m_q.evolve(m_qdot, qddot, DT);
        // Actual position within limits
        q = m_arm.feasibility().clamp(q);
        // Implied accel for feasible position and previous velocity
        qddot = RRRAcceleration.solve(m_q, q, m_qdot, DT);
        // Implied velocity for feasible position, accel, and previous velocity
        qdot = RRRVelocity.evolve(m_qdot, qddot, DT);
        m_q = q;
        m_qdot = qdot;
        m_arm.set(m_q, m_qdot, qddot);
    }

}
