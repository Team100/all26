package org.team100.lib.subsystems.swerve.commands.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.reference.se2.ProfileReferenceSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Rotate in place to the specified angle.
 * 
 * This is probably only useful for testing.
 */
public class Rotate extends MoveAndHold {
    /** For testing */
    private static final boolean DEBUG = false;
    private static final double THETA_TOLERANCE_RAD = 0.02;
    // don't try to rotate at max speed
    private static final double SPEED = 0.5;

    private final LoggerFactory m_log;
    private final SwerveDriveSubsystem m_drive;
    private final ControllerSE2 m_controller;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final Rotation2d m_target;
    private final ProfileSE2 m_profile;

    private ProfileReferenceSE2 m_reference;
    private VelocityReferenceControllerSE2 m_referenceController;

    public Rotate(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            ControllerSE2 controller,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        m_log = parent.type(this);
        m_drive = drive;
        m_controller = controller;
        m_swerveKinodynamics = swerveKinodynamics;
        m_target = new Rotation2d(targetAngleRadians);
        m_profile = HolonomicProfileFactory.trapezoidal(
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2(),
                0.01,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * SPEED,
                m_swerveKinodynamics.getMaxAngleAccelRad_S2() * SPEED,
                THETA_TOLERANCE_RAD);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (DEBUG)
            System.out.println("Rotate initialize");
        Pose2d measurement = m_drive.getPose();
        // if we use the initial measurement x and y as the target, and we're moving,
        // then we make a u-turn to get back to the arbitrary place when we pushed the
        // button.
        // instead, pick a goal at the stopping distance in the current direction.
        Translation2d dx = m_drive.getVelocity().stopping(m_swerveKinodynamics.getMaxDriveAccelerationM_S2());
        Pose2d goal = new Pose2d(measurement.getX() + dx.getX(), measurement.getY() + dx.getY(), m_target);
        m_reference = new ProfileReferenceSE2(m_log, m_profile, "rotate");
        m_reference.setGoal(new ModelSE2(goal));
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        if (DEBUG)
            System.out.println("Rotate execute");
        if (m_referenceController != null)
            m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean isInterupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
    }
}
