package org.team100.lib.subsystems.rrr.commands;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.rrr.RRRArm;
import org.team100.lib.subsystems.se2.commands.helper.PositionReferenceControllerSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Plan a trajectory in SE2 (the usual swerve way), and then follow it in SE2,
 * using inverse kinematics at each step.
 * 
 * * The benefit of this method is that it follows a workspace path.
 * 
 * * The drawback of this method is that it is sensitive to interior
 * singularities and joint limits.
 * 
 * There could be some sort of hybrid follower for the joint limits -- recognize
 * when you hit one, and spin to the other side of it, like the "ballerina" demo
 * does, but that would be complicated and error prone.
 */
public class MoveWithTrajectorySE2 extends MoveAndHold {
    private final LoggerFactory m_log;
    private final RRRArm m_arm;
    private final Pose2d m_goal;
    private final TrajectorySE2Planner m_planner;
    /** Non-null when the command is running, otherwise null. */
    private PositionReferenceControllerSE2 m_referenceController;

    public MoveWithTrajectorySE2(LoggerFactory parent, RRRArm arm, Pose2d goal, double speed) {
        m_log = parent.type(this);
        m_arm = arm;
        m_goal = goal;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(speed, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Pose2d start = m_arm.pose();
        Translation2d currTranslation = start.getTranslation();
        Rotation2d courseToGoal = m_goal.getTranslation().minus(currTranslation).getAngle();

        List<WaypointSE2> waypoints = new ArrayList<>();
        waypoints.add(new WaypointSE2(start, DirectionSE2.irrotational(courseToGoal), 1));
        waypoints.add(new WaypointSE2(m_goal, DirectionSE2.irrotational(courseToGoal), 1));

        TrajectorySE2 trajectory = m_planner.restToRest(waypoints);
        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(m_log, trajectory);
        m_referenceController = new PositionReferenceControllerSE2(
                m_log, m_arm, reference);
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
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();

    }

}
