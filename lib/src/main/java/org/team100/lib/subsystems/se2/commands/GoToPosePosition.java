package org.team100.lib.subsystems.se2.commands;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.PositionReferenceControllerSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;

/**
 * Using the pose at initialization time, and the specified course, construct a
 * rest-to-rest trajectory to the goal and follow it.
 */
public class GoToPosePosition extends MoveAndHold {
    private final LoggerFactory m_log;
    private final PositionSubsystemSE2 m_subsystem;
    private final WaypointSE2 m_goal;
    private final DirectionSE2 m_course;
    private final TrajectorySE2Planner m_planner;

    private PositionReferenceControllerSE2 m_referenceController;

    public GoToPosePosition(
            LoggerFactory parent,
            PositionSubsystemSE2 subsystem,
            DirectionSE2 course,
            WaypointSE2 goal,
            TrajectorySE2Planner planner) {
        m_log = parent.type(this);
        m_subsystem = subsystem;
        m_goal = goal;
        m_course = course;
        m_planner = planner;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        WaypointSE2 m_currentPose = new WaypointSE2(
                m_subsystem.getState().pose(),
                m_course, 1);
        TrajectorySE2 m_trajectory = m_planner.restToRest(
                List.of(m_currentPose, m_goal));
        m_referenceController = new PositionReferenceControllerSE2(
                m_log, m_subsystem, 0.1, new TrajectoryReferenceSE2(m_log, m_trajectory));
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        if (m_referenceController == null)
            return false;
        return m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }

}