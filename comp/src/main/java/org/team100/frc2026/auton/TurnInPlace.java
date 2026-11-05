package org.team100.frc2026.auton;

import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.SubsystemSE2;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnInPlace implements AnnotatedCommand {
    private final SubsystemSE2 m_drive;
    private final double m_radians;
    private final Command m_command;

    public TurnInPlace(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery,
            double radians) {
        LoggerFactory log = parent.name(name());
        m_drive = machinery.m_drive;
        m_radians = radians;
        ProfileSE2 profile = HolonomicProfileFactory.unwrappedTrapezoidal(
                3, 10, 0.01, 3, 10, 0.01);
        DriveToPoseWithProfile dtpwp = new DriveToPoseWithProfile(
                log, machinery.m_drive, controller, profile, this::goal);
        m_command = dtpwp.until(dtpwp::isDone);
    }

    /** Called once at initialization. */
    private Pose2d goal() {
        Pose2d currentPose = m_drive.getState().pose();
        // This isn't just Pose.plus() because we want multiple turns.
        return new Pose2d(
                currentPose.getX(),
                currentPose.getY(),
                new Rotation2d(currentPose.getRotation().getRadians() + m_radians));
    }

    @Override
    public String name() {
        return String.format("Turn In Place %5.2f", m_radians);
    }

    @Override
    public Command command() {
        return m_command;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        // No trajectory to show for this command.
        return List.of();
    }

}
