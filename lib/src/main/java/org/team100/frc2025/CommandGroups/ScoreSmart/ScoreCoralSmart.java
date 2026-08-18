package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;

import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.field.FieldConstants2025;
import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.swerve.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoralSmart {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            ControllerSE2 controller,
            ProfileSE2 profile,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            Supplier<ScoringLevel> level,
            Supplier<ReefPoint> point) {
        Supplier<Pose2d> goal = () -> FieldConstants2025.makeGoal(level.get(), point.get());
        return parallel(
                runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                select(Map.ofEntries(
                        Map.entry(ScoringLevel.L4,
                                ScoreL4Smart.get(
                                        logger, mech, manipulator,
                                        controller, profile, drive, goal)),
                        Map.entry(ScoringLevel.L3,
                                ScoreL3Smart.get(
                                        logger, mech, manipulator,
                                        controller, profile, drive, goal)),
                        Map.entry(ScoringLevel.L2,
                                ScoreL2Smart.get(
                                        logger, mech, manipulator,
                                        controller, profile, drive, goal)),
                        Map.entry(ScoringLevel.L1,
                                ScoreL1Smart.get(
                                        logger, mech, manipulator,
                                        controller, profile, drive, goal)),
                        Map.entry(ScoringLevel.NONE, print("No button pressed"))),
                        level))
                .withName("ScoreCoralSmart");
    }
}
