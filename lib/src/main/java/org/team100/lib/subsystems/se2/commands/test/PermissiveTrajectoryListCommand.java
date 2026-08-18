package org.team100.lib.subsystems.se2.commands.test;

import java.util.Iterator;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Similar to TrajectoryListCommand, but each trajectory starts wherever the
 * robot ends up, instead of at the end of the previous trajectory. This is
 * essentially like ignoring cross-track error.
 */
public class PermissiveTrajectoryListCommand extends MoveAndHold {
    private final LoggerFactory m_log;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final List<Function<Pose2d, TrajectorySE2>> m_trajectories;
    private final TrajectoryVisualization m_viz;

    private Iterator<Function<Pose2d, TrajectorySE2>> m_trajectoryIter;
    private VelocityReferenceControllerSE2 m_referenceController;

    public PermissiveTrajectoryListCommand(
            LoggerFactory parent,
            VelocitySubsystemSE2 swerve,
            ControllerSE2 controller,
            List<Function<Pose2d, TrajectorySE2>> trajectories,
            TrajectoryVisualization viz) {
        m_log = parent.type(this);
        m_drive = swerve;
        m_controller = controller;
        m_trajectories = trajectories;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectoryIter = m_trajectories.iterator();
        m_referenceController = null;
    }

    @Override
    public void execute() {
        if (m_referenceController == null || m_referenceController.isDone()) {
            // get the next trajectory
            if (m_trajectoryIter.hasNext()) {
                TrajectorySE2 trajectory = m_trajectoryIter.next().apply(m_drive.getState().pose());
                TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(m_log, trajectory);
                m_referenceController = new VelocityReferenceControllerSE2(
                        m_log, m_drive, m_controller, reference);
                m_viz.setViz(trajectory);
            } else {
                return;
            }
        }

        // now there is a trajectory to follow
        if (m_referenceController != null)
            m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController == null || m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }

}
