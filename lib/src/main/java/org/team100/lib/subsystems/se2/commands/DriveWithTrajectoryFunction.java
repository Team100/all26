package org.team100.lib.subsystems.se2.commands;

import java.util.function.Function;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.visualization.TrajectoryVisualization;

import org.wpilib.math.geometry.Pose2d;

/**
 * Follow a trajectory created at initialization time, given the pose at that
 * time. Since the trajectory function takes a pose, and not a state, then
 * probably the returned trajectory should start from rest.
 */
public class DriveWithTrajectoryFunction extends MoveAndHold {
    private final LoggerFactory m_log;
    private final BooleanLogger m_logDone;
    private final DoubleLogger m_logToGo;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final TrajectoryVisualization m_viz;
    private final Function<Pose2d, TrajectorySE2> m_trajectoryFn;

    /**
     * Non-null when the command is active (between initialize and end), null
     * otherwise.
     */
    private VelocityReferenceControllerSE2 m_referenceController;

    public DriveWithTrajectoryFunction(
            LoggerFactory parent,
            VelocitySubsystemSE2 drive,
            ControllerSE2 controller,
            TrajectoryVisualization viz,
            Function<Pose2d, TrajectorySE2> trajectoryFn) {
        m_log = parent.type(this);
        m_logDone = m_log.booleanLogger(Level.DEBUG, "done");
        m_logToGo = m_log.doubleLogger(Level.TRACE, "to go");
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_trajectoryFn = trajectoryFn;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        TrajectorySE2 trajectory = m_trajectoryFn.apply(m_drive.getState().pose());
        m_viz.setViz(trajectory);
        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(m_log, trajectory);
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
        toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        boolean done = m_referenceController != null && m_referenceController.isDone();
        m_logDone.log(() -> done);
        return done;
    }

    @Override
    public double toGo() {
        double togo = (m_referenceController == null) ? 0 : m_referenceController.toGo();
        m_logToGo.log(() -> togo);
        return togo;
    }

}
