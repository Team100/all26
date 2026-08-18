package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreL1Smart {
    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            ControllerSE2 controller,
            ProfileSE2 profile,
            SwerveDriveSubsystem drive,
            Supplier<Pose2d> goal) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, drive, controller, profile, goal);
        MoveAndHold toL1 = mech.homeToL1();
        Command eject = manipulator.centerEject().withTimeout(0.5);
        return sequence(
                parallel(
                        toReef,
                        waitUntil(() -> toReef.toGo() < 1)
                                .andThen(toL1),
                        waitUntil(() -> (toReef.isDone() && toL1.isDone()))
                                .andThen(eject)//
                ).until(() -> (toReef.isDone() && toL1.isDone() && eject.isFinished())),
                mech.l1ToHome()//
        );
    }
}
