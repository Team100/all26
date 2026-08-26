package org.team100.lib.subsystems.se2.commands.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.ConstantReferenceSE2;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;

/**
 * Given a swerve state, drive there using only the feedback in the holonomic
 * controller.
 * 
 * There's no profile, no trajectory, just give it a point
 * you want to go to, and it will go there. It makes no attempt to
 * impose feasibility constraints or coordinate the axes.
 * 
 * This is really only intended for testing.
 */
public class DriveToStateSimple extends MoveAndHold {
    private final LoggerFactory m_log;
    private final ControllerSE2 m_controller;
    private final VelocitySubsystemSE2 m_drive;
    private final ReferenceSE2 m_reference;

    private VelocityReferenceControllerSE2 m_referenceController;

    public DriveToStateSimple(
            LoggerFactory parent,
            ControllerSE2 controller,
            VelocitySubsystemSE2 drive,
            StateSE2 goal) {
        m_log = parent.type(this);
        m_controller = controller;
        m_drive = drive;
        m_reference = new ConstantReferenceSE2(goal);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, m_reference);
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
        m_drive.stop();
    }
}
