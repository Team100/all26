package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

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
import edu.wpi.first.wpilibj2.command.Command;

/** An example of a simple sequence */
public class CenterPreloadedAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public CenterPreloadedAuton(
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
        return "Shoot from Center";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.SHOOT_CENTER,
                        new DirectionSE2(-1, 0, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction ScoreSetUp = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);

        // Shoot preloaded balls
        return sequence(
                parallel(
                        ScoreSetUp.until(ScoreSetUp::isDone).withTimeout(3.5),
                        machinery.m_shooter.auto()),
                waitSeconds(5),
                machinery.m_shooter.stop().withTimeout(1));
    }

    @Override
    public Pose2d start() {
        return AutonPositions.CENTER;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }

}
