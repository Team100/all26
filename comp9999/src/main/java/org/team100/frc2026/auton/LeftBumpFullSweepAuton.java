package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.constraint.VelocityLimitRegionConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** An example of a simple sequence */
public class LeftBumpFullSweepAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public LeftBumpFullSweepAuton(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        constraints = new TimingConstraintFactory(kinodynamics).auto();
        // In meters/second
        double maxBumpVelocity = 2;
        List<TimingConstraint> new_constraints = new ArrayList<>(constraints);

        // create a new VelocityRegionContstraint `slow_bump_zone`
        VelocityLimitRegionConstraint slow_bump_zone = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone2 = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_RIGHT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone3 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone4 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_RIGHT, maxBumpVelocity);
        new_constraints.add(slow_bump_zone);
        new_constraints.add(slow_bump_zone2);
        new_constraints.add(slow_bump_zone3);
        new_constraints.add(slow_bump_zone4);
        // constraints.add(slow_bump_zone);
        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "Full Sweep from Left Bump";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, -0.2, 0), 1),
                new WaypointSE2(new Pose2d(7.75, 7, new Rotation2d(90 * (Math.PI / 180))),
                        new DirectionSE2(0.1, -1, 0), 1),
                new WaypointSE2(new Pose2d(7.75, 1, new Rotation2d(90 * (Math.PI / 180))),
                        new DirectionSE2(0, -1, 0), 1),
                new WaypointSE2(new Pose2d(7, 4, new Rotation2d(270 * (Math.PI / 180))),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(StartingPositions.LEFT_BUMP,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.SHOOT_LEFT,
                        new DirectionSE2(-1, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t2(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, -0.2, 0), 1),
                new WaypointSE2(AutonPositions.LEFT_BUMP_180DEG,
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(new Pose2d(7.5, 4, new Rotation2d(90 * (Math.PI / 180))),
                        new DirectionSE2(0, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction Cycle1 = new DriveWithTrajectoryFunction(
                log.name("Cycle1"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction Cycle2 = new DriveWithTrajectoryFunction(
                log.name("Cycle1"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);

        // Intake, score
        return sequence(
                parallel(
                        Cycle1.until(Cycle1::isDone).withTimeout(8),
                        sequence(
                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInNeutralZone(machinery.m_drive.getState().translation())),
                                (machinery.m_intakeExtend.goToExtendedPosition()
                                        .andThen(machinery.m_intake.intake())).withTimeout(4),

                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInAllianceZone(machinery.m_drive.getState().translation())),
                                parallel(
                                        machinery.m_intake.stop(),
                                        // machinery.m_intakeExtend.goToRetractedPosition(),
                                        machinery.m_shooter.auto())
                                        .withTimeout(4))),
                parallel(
                        Cycle2.until(Cycle2::isDone),
                        Commands.waitUntil(() -> FieldConstants2026
                                .isInNeutralZone(machinery.m_drive.getState().translation()))
                                .andThen(
                                        machinery.m_intake.intake())));
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1, this::t2);
    }

}
