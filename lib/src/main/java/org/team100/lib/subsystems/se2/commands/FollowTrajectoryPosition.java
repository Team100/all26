package org.team100.lib.subsystems.se2.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.PositionReferenceControllerSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;

/** Analogous to DriveWithTrajectory, but for SE2 positional control. */
public class FollowTrajectoryPosition extends MoveAndHold {
    private final LoggerFactory m_log;
    private final PositionSubsystemSE2 m_subsystem;
    private final TrajectorySE2 m_trajectory;

    private PositionReferenceControllerSE2 m_referenceController;

    public FollowTrajectoryPosition(
            LoggerFactory parent,
            PositionSubsystemSE2 subsystem,
            TrajectorySE2 trajectory) {
        m_log = parent.type(this);
        m_subsystem = subsystem;
        m_trajectory = trajectory;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_referenceController = new PositionReferenceControllerSE2(
                m_log, m_subsystem, 0.1, new TrajectoryReferenceSE2(m_log, m_trajectory));
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
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
