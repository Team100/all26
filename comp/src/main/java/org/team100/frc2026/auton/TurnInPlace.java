package org.team100.frc2026.auton;

import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.profile.se2_universal_cover.WindingProfileSE2;
import org.team100.lib.subsystems.se2.SubsystemSE2;
import org.team100.lib.subsystems.se2.commands.DriveToWindingPose2dWithProfile;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;

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
        m_radians = radians;
        LoggerFactory log = parent.name(name());
        m_drive = machinery.m_drive;
        WindingProfileSE2 profile = HolonomicProfileFactory.windingTrapezoidal(
                3, 10, 0.01, 3, 10, 0.01);
        DriveToWindingPose2dWithProfile dtpwp = new DriveToWindingPose2dWithProfile(
                log, machinery.m_drive, controller, profile, this::goal);
        m_command = dtpwp.until(dtpwp::isDone);
    }

    /** Called once at initialization. */
    private WindingPose2d goal() {
        Pose2d currentPose = m_drive.getState().pose();
        // This isn't just Pose.plus() because we want multiple turns.
        return new WindingPose2d(
                currentPose.getX(),
                currentPose.getY(),
                currentPose.getRotation().getRadians() + m_radians);
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
