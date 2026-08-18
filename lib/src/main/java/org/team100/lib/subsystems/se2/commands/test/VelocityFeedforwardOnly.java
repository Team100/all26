package org.team100.lib.subsystems.se2.commands.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.reference.se2.ProfileReferenceSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Use a profile with feedforward control only.
 * 
 * This is mainly useful for testing.
 */
public class VelocityFeedforwardOnly extends MoveAndHold {
    private static final boolean DEBUG = false;

    private final LoggerFactory m_log;
    private final ProfileSE2 m_profile;
    private final Pose2d m_goal;
    private final VelocitySubsystemSE2 m_drive;

    private ProfileReferenceSE2 m_reference;

    public VelocityFeedforwardOnly(
            LoggerFactory parent,
            ProfileSE2 profile,
            Pose2d goal,
            VelocitySubsystemSE2 drive) {
        m_log = parent.type(this);
        m_profile = profile;
        m_goal = goal;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_reference = new ProfileReferenceSE2(m_log, m_profile, "feedforward only");
        m_reference.setGoal(new ModelSE2(m_goal));
        m_reference.initialize(m_drive.getState());
    }

    @Override
    public void execute() {
        VelocityControlSE2 velocity = m_reference.next().velocityControl();
        if (DEBUG)
            System.out.printf("velocity %s\n", velocity);
        m_drive.set(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
    }

    @Override
    public boolean isDone() {
        return m_reference.done();
    }

    @Override
    public double toGo() {
        ModelSE2 goal = m_reference.goal();
        ModelSE2 measurement = m_drive.getState();
        return goal.minus(measurement).translation().getNorm();
    }
}
