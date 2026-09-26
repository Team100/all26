package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example of a simple sequence */
public class LeftSequenceExample implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public LeftSequenceExample(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        constraints = new TimingConstraintFactory(kinodynamics).auto();
        trajectoryFactory = new TrajectorySE2Factory(constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "Left Sequence Example";
    }

    // go to the middle while spinning 90 degrees
    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(new Pose2d(2, 4, Rotation2d.kCW_Pi_2),
                        new DirectionSE2(0, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    // go back where we started.
    TrajectorySE2 t2(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(StartingPositions.LEFT_TRENCH,
                        new DirectionSE2(1, 0, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction n2 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);
        return sequence(
                n1.until(n1::isDone),
                waitSeconds(1),
                n2.until(n2::isDone));
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_TRENCH;
    }

        @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1, this::t2);
    }

}
