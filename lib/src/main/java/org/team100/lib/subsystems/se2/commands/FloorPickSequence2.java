package org.team100.lib.subsystems.se2.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.targeting.Targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a sequence of commands to drive nearby a target with the intake facing
 * it, and then sweep the intake over the target.
 * 
 * Similar to FloorPickSequence, but since that was modified by Vasili and
 * Marcelo, and I want to leave it alone.
 */
public class FloorPickSequence2 {
    /** End effector relative to robot */
    private static final Transform2d END_EFFECTOR = new Transform2d(-0.5, 0.193, Rotation2d.k180deg);

    public static Command get(
            LoggerFactory parent,
            LoggerFactory fieldLog,
            VelocitySubsystemSE2 drive,
            Targets targets,
            ControllerSE2 controller,
            ProfileSE2 profile) {
        LoggerFactory log = parent.type(new FloorPickSequence());
        Pushbroom toRunway = new Pushbroom(
                log, fieldLog, targets::getClosestTarget, drive,
                controller, profile, END_EFFECTOR);

        return sequence(toRunway.until(toRunway::isDone));
    }

}
