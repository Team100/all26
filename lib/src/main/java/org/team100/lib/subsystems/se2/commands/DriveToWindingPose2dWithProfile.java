package org.team100.lib.subsystems.se2.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.WindingPose2dLogger;
import org.team100.lib.profile.se2_universal_cover.WindingProfileSE2;
import org.team100.lib.reference.se2_universal_cover.WindingProfileReferenceSE2;
import org.team100.lib.state.WindingStateSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;

/**
 * Drive to a pose supplied at initialization, using a profile.
 */
public class DriveToWindingPose2dWithProfile extends MoveAndHold {
    private final LoggerFactory m_log;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final WindingProfileSE2 m_profile;
    private final WindingPose2dLogger m_log_goal;
    private final Supplier<WindingPose2d> m_goal;

    private WindingProfileReferenceSE2 m_reference;
    private VelocityReferenceControllerSE2 m_referenceController;

    public DriveToWindingPose2dWithProfile(
            LoggerFactory logger,
            VelocitySubsystemSE2 drive,
            ControllerSE2 controller,
            WindingProfileSE2 profile,
            Supplier<WindingPose2d> goal) {
        m_log = logger.type(this);
        m_log_goal = m_log.WindingPose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_goal = goal;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        WindingPose2d goal = m_goal.get();
        m_log_goal.log(() -> goal);
        m_reference = new WindingProfileReferenceSE2(m_log, m_profile, "embark");
        m_reference.setGoal(new WindingStateSE2(goal));
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
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
