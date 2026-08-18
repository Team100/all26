package org.team100.lib.subsystems.six_dof.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rn.WaypointRn;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.rn.PositionReferenceControllerRn;
import org.team100.lib.reference.rn.SplineReferenceRn;
import org.team100.lib.spline.rn.SplineRn;
import org.team100.lib.subsystems.six_dof.SixDofArm;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N6;

/** Generate a spline in R6 joint space, and follow it. */
public class MoveWithSpline extends MoveAndHold {
    private static final boolean DEBUG = false;
    private final LoggerFactory m_log;
    private final SixDofArm m_arm;
    private final VelocitySE3 m_x0dot;
    private final Pose3d m_x1;
    private final VelocitySE3 m_x1dot;
    /** Non-null when the command is running, otherwise null. */
    private PositionReferenceControllerRn m_referenceController;

    public MoveWithSpline(
            LoggerFactory parent,
            SixDofArm arm,
            VelocitySE3 x0dot,
            Pose3d x1,
            VelocitySE3 x1dot) {
        m_log = parent.type(this);
        m_arm = arm;
        m_x0dot = x0dot;
        m_x1 = x1;
        m_x1dot = x1dot;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_x1) == null)
            throw new IllegalArgumentException("infeasible goal");

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        SixDofConfig q0 = m_arm.getConfig();
        SixDofConfig q1 = m_arm.config(m_x1);

        SixDofVelocity q0dot = m_arm.qdot(q0, m_x0dot);
        SixDofVelocity q1dot = m_arm.qdot(q1, m_x1dot);

        WaypointRn<N6> p0 = new WaypointRn<>(q0.toVector(), q0dot.toVector());
        WaypointRn<N6> p1 = new WaypointRn<>(q1.toVector(), q1dot.toVector());

        if (DEBUG) {
            System.out.printf("p0 %s p1 %s\n", p0, p1);
        }

        SplineRn<N6> spline = new SplineRn<>(Nat.N6(), p0, p1);

        // duration respects joint distances.
        // double qdistance = Metrics.l1Norm(q0.toVector().minus(q1.toVector()));
        double qdistance = q0.distance(q1);
        double duration = 0.5 * qdistance;
        if (DEBUG)
            System.out.printf("duration %f\n", duration);
        SplineReferenceRn<N6> reference = new SplineReferenceRn<>(
                spline, duration);
        m_referenceController = new PositionReferenceControllerRn(
                m_arm, reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        SixDofConfig q0 = m_arm.getConfig();
        SixDofConfig q1 = m_arm.config(m_x1);
        return q0.distance(q1);
    }

}
