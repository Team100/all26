package org.team100.frc2025.robot;

import static org.wpilib.command2.Commands.parallel;
import static org.wpilib.command2.Commands.runOnce;
import static org.wpilib.command2.Commands.sequence;
import static org.wpilib.command2.Commands.waitUntil;

import java.util.List;

import org.team100.frc2025.field.FieldConstants2025;
import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.se2.FullStateControllerSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.se2.ProfileSE2;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.se2.commands.DriveToTranslationFacingWithProfile;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.ParallelRaceGroup;

public class LolipopAuto {
    private static final double HEED_RADIUS_M = 3;
    private final LoggerFactory m_log;
    private final Machinery2025 m_machinery;
    private final ProfileSE2 m_autoProfile;
    private final FullStateControllerSE2 m_autoController;
    private final TrajectorySE2Planner m_planner;

    public LolipopAuto(
            LoggerFactory parent,
            Machinery2025 machinery,
            ProfileSE2 autoProfile,
            FullStateControllerSE2 autoController,
            TrajectorySE2Planner planner) {
        m_log = parent.type(this);
        m_machinery = machinery;
        m_autoProfile = autoProfile;
        m_autoController = autoController;
        m_planner = planner;
    }

    public Command get() {
        // this one uses some curvature
        DriveWithTrajectoryFunction toReefTrajectory = new DriveWithTrajectoryFunction(
                m_log, m_machinery.m_drive, m_autoController, m_machinery.m_trajectoryViz,
                (p) -> m_planner.restToRest(List.of(
                        WaypointSE2.irrotational(m_machinery.m_drive.getPose(), Math.PI, 1.2),
                        WaypointSE2.irrotational(
                                new Pose2d(3, 5, new Rotation2d(0)), -2, 1.2))));

        DriveToPoseWithProfile toReefA = new DriveToPoseWithProfile(
                m_log, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants2025.makeGoal(ScoringLevel.L4, ReefPoint.A));

        DriveToTranslationFacingWithProfile toCenterCoral = new DriveToTranslationFacingWithProfile(
                m_log, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants2025.CoralMark.CENTER.value
                        .plus(new Translation2d(0.7, 0)),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefB = new DriveToPoseWithProfile(
                m_log, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants2025.makeGoal(ScoringLevel.L4, ReefPoint.B));

        DriveToTranslationFacingWithProfile toCoralRight = new DriveToTranslationFacingWithProfile(
                m_log, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants2025.CoralMark.RIGHT.value
                        .plus(new Translation2d(0.7, 0).rotateBy(new Rotation2d(Math.PI / 4))),
                new Rotation2d(Math.PI));

        DriveToPoseWithProfile toReefC = new DriveToPoseWithProfile(
                m_log, m_machinery.m_drive, m_autoController, m_autoProfile,
                () -> FieldConstants2025.makeGoal(ScoringLevel.L4, ReefPoint.C));

        MoveAndHold toL4 = m_machinery.m_mech.homeToL4();
        MoveAndHold toL4second = m_machinery.m_mech.homeToL4();
        MoveAndHold toL4third = m_machinery.m_mech.homeToL4();
        ParallelRaceGroup eject = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectsecond = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        ParallelRaceGroup ejectthird = m_machinery.m_manipulator.centerEject().withTimeout(0.3);
        return sequence(
                toReefTrajectory.until(toReefTrajectory::isDone),
                parallel(
                        runOnce(() -> m_machinery.m_localizer.setHeedRadiusM(HEED_RADIUS_M)),
                        toReefA,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefA::isDone).andThen(toL4),
                        waitUntil(() -> toReefA.isDone() && toL4.isDone())
                                .andThen(eject))
                        .until(() -> (toReefA.isDone() && toL4.isDone() && eject.isFinished())),
                parallel(
                        toCenterCoral,
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()).until(m_machinery.m_manipulator::hasCoral)
                        .withTimeout(3)
                        .andThen(m_machinery.m_manipulator::stop),
                parallel(
                        toReefB,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefB::isDone).andThen(toL4second),
                        waitUntil(() -> toReefB.isDone() && toL4second.isDone())
                                .andThen(ejectsecond))
                        .until(() -> (toReefB.isDone() && toL4second.isDone() && ejectsecond.isFinished())),
                parallel(
                        toCoralRight,
                        m_machinery.m_mech.pickWithProfile(),
                        m_machinery.m_manipulator.centerIntake()).until(m_machinery.m_manipulator::hasCoral)
                        .withTimeout(3)
                        .andThen(m_machinery.m_manipulator::stop),
                parallel(
                        toReefC,
                        m_machinery.m_mech.profileHomeAndThenRest().until(toReefC::isDone).andThen(toL4third),
                        waitUntil(() -> toReefC.isDone() && toL4third.isDone())
                                .andThen(ejectthird))
                        .until(() -> (toReefC.isDone() && toL4third.isDone() && ejectthird.isFinished())),
                m_machinery.m_mech.l4ToHome());
    }
}
