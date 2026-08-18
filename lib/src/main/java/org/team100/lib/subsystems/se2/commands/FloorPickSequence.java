package org.team100.lib.subsystems.se2.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.targeting.Targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a sequence of commands to drive nearby a target with the intake facing
 * it, and then sweep the intake over the target.
 */
public class FloorPickSequence {
    // landing point distance to target
    private static final double DISTANCE = 0.5;
    private static final Translation2d PICKOFFSET = new Translation2d(0, 0.193); // pick is from the back
    private static final Rotation2d RELATIVE_BEARING = Rotation2d.k180deg;

    public static Command get(
            LoggerFactory parent,
            LoggerFactory fieldLog,
            VelocitySubsystemSE2 drive,
            Targets targets,
            ControllerSE2 controller,
            ProfileSE2 profile) {
        LoggerFactory log = parent.type(FloorPickSequence.class);
        Supplier<Optional<Translation2d>> target = targets::getClosestTarget;
        Supplier<Optional<Translation2d>> runway = () -> {
            Optional<Translation2d> t = target.get();
            if (t.isEmpty())
                return t;
            Translation2d goal = t.get();
            Rotation2d course = goal.minus(drive.getState().pose().getTranslation()).getAngle();
            Translation2d landing = goal.minus(new Translation2d(DISTANCE, course));
            return Optional.of(landing);
        };

        DriveToTranslationWithRelativeBearing toRunway = new DriveToTranslationWithRelativeBearing(
                log, fieldLog, runway, drive,
                controller, profile, RELATIVE_BEARING, PICKOFFSET);

        return sequence(
                toRunway.until(toRunway::isDone));
    }

}
