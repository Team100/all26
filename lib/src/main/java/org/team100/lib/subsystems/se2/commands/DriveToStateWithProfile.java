package org.team100.lib.subsystems.se2.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.reference.se2.ProfileReferenceSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;

/**
 * Drive to the supplied goal state using a profile. Allows the goal to change
 * after initialization.
 */
public class DriveToStateWithProfile extends MoveAndHold {
    private final LoggerFactory m_log;
    private final DoubleArrayLogger m_log_target;
    private final Supplier<StateSE2> m_goals;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final ProfileSE2 m_profile;

    private StateSE2 m_goal;
    private ProfileReferenceSE2 m_reference;
    private VelocityReferenceControllerSE2 m_referenceController;

    public DriveToStateWithProfile(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            Supplier<StateSE2> goal,
            VelocitySubsystemSE2 drive,
            ControllerSE2 controller,
            ProfileSE2 profile) {
        m_log = parent.type(this);
        m_log_target = fieldLogger.doubleArrayLogger(Level.TRACE, "target");
        m_goals = goal;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_goal = m_goals.get();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceSE2(m_log, m_profile, "Drive to pose with profile");
        m_reference.setGoal(m_goal);
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        m_goal = m_goals.get();
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(m_goal);
        m_referenceController.execute();
        m_log_target.log(() -> new double[] {
                m_goal.x().x(),
                m_goal.y().x(),
                m_goal.theta().x() });
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
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
