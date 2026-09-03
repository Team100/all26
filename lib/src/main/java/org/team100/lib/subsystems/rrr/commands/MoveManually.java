package org.team100.lib.subsystems.rrr.commands;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.subsystems.rrr.RRRArm;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;



public class MoveManually extends Command {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final RRRArm m_arm;
    private final Gamepad m_controller;
    private RRRConfig m_q;
    private RRRVelocity m_qdot = new RRRVelocity(0, 0, 0);

    public MoveManually(RRRArm arm, Gamepad controller) {
        m_arm = arm;
        m_controller = controller;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_q = m_arm.getConfig();
    }

    @Override
    public void execute() {
        // controller specifies velocity
        double q1dot = -15.0 * m_controller.getLeftX(); // axis 0
        double q2dot = -5.0 * m_controller.getRightX(); // axis 4
        double q3dot = -5.0 * m_controller.getRightY(); // axis 5
        RRRVelocity qdot = new RRRVelocity(q1dot, q2dot, q3dot);
        // integrate to find q
        // x = v * dt
        RRRConfig dq = RRRConfig.fromVector(qdot.toVector().times(DT));
        RRRConfig q = m_q.plus(dq);
        // differentiate to find qddot
        Vector<N3> dqdot = qdot.toVector().minus(m_qdot.toVector());
        // a = dv/dt
        RRRAcceleration qddot = RRRAcceleration.fromVector(dqdot.div(DT));
        m_q = q;
        m_qdot = qdot;
        m_arm.set(m_q, m_qdot, qddot);
        m_q = m_arm.getConfigWithinLimits();
    }

}
