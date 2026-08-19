package org.team100.frc2025.CommandGroups.ScoreSmart;

import static org.wpilib.command2.Commands.parallel;
import static org.wpilib.command2.Commands.sequence;
import static org.wpilib.command2.Commands.waitUntil;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;

public class ScoreL3Smart {
    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            ControllerSE2 controller,
            ProfileSE2 profile,
            SwerveDriveSubsystem m_drive,
            Supplier<Pose2d> goal) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, m_drive, controller, profile, goal);
        MoveAndHold toL3 = mech.homeToL3();
        Command eject = manipulator.centerEject().withTimeout(0.5);
        return sequence(
                parallel(
                        toReef,
                        waitUntil(() -> toReef.toGo() < 1)
                                .andThen(toL3),
                        waitUntil(() -> (toReef.isDone() && toL3.isDone()))
                                .andThen(eject) //
                ).until(() -> (toReef.isDone() && toL3.isDone() && eject.isFinished())),
                mech.l3ToHome() //
        );
    }
}
