package org.team100.lib.subsystems.rr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rn.WaypointRn;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.rn.PositionReferenceControllerRn;
import org.team100.lib.reference.rn.SplineReferenceRn;
import org.team100.lib.spline.rn.SplineRn;
import org.team100.lib.subsystems.rr.RRArm;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

/**
 * Generate a spline in R3, in joint space, and follow it.
 * 
 * The starting point is the current configuration.
 * 
 * The endpoint is specified.
 * 
 * The velocities at each end are specified. Note: specifying the starting
 * velocity really only makes sense if you kinda know where the starting point
 * really is, i.e. for sequences.
 * 
 * TODO: make another one of these where the initial velocity is computed,
 * e.g. towards the endpoint, or something.
 * 
 * * The benefit of using joint splines is that it is immune to interior
 * singularities and joint limits.
 * 
 * * The drawback is that it doesn't follow any particular workspace path --
 * it's better than the simple profiled method, due to the care in choosing
 * endpoint directions, but in between, there's nothing to make the path do
 * anything in particular (e.g. be straight).
 */
public class MoveWithSpline extends MoveAndHold {
    private static final boolean DEBUG = false;
    @SuppressWarnings("unused")
    private final LoggerFactory m_log;
    private final RRArm m_arm;
    private final VelocityR2 m_x0dot;
    private final Translation2d m_x1;
    private final VelocityR2 m_x1dot;
    /** Non-null when the command is running, otherwise null. */
    private PositionReferenceControllerRn<N2> m_referenceController;

    public MoveWithSpline(
            LoggerFactory parent,
            RRArm arm,
            VelocityR2 x0dot,
            Translation2d x1,
            VelocityR2 x1dot) {
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
        RRConfig q0 = m_arm.getConfig();
        RRConfig q1 = m_arm.config(m_x1);

        RRVelocity q0dot = m_arm.qdot(q0, m_x0dot);
        RRVelocity q1dot = m_arm.qdot(q1, m_x1dot);

        WaypointRn<N2> p0 = new WaypointRn<>(q0.toVector(), q0dot.toVector());
        WaypointRn<N2> p1 = new WaypointRn<>(q1.toVector(), q1dot.toVector());

        if (DEBUG) {
            System.out.printf("p0 %s p1 %s\n", p0, p1);
        }

        SplineRn<N2> spline = new SplineRn<>(Nat.N2(), p0, p1);

        // duration respects joint distances.
        // double qdistance = Metrics.l1Norm(q0.toVector().minus(q1.toVector()));
        double qdistance = q0.distance(q1);
        double duration = 0.5 * qdistance;
        if (DEBUG)
            System.out.printf("duration %f\n", duration);
        SplineReferenceRn<N2> reference = new SplineReferenceRn<>(
                spline, duration);
        m_referenceController = new PositionReferenceControllerRn<>(
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
        RRConfig q0 = m_arm.getConfig();
        RRConfig q1 = m_arm.config(m_x1);
        return q0.distance(q1);
    }
}
