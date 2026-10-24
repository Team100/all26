package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

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
public class AutonTest2 implements AnnotatedCommand {
    private final LoggerFactory log;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public AutonTest2(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        double maxBumpVelocity = 1.6;
        List<TimingConstraint> new_constraints = Stream.concat(
                new TimingConstraintFactory(kinodynamics).auto().stream(),
                BumpZones.constraint(maxBumpVelocity).stream()).toList();

        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction n2 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);
        command = sequence(
                n1.until(n1::isDone),
                waitSeconds(1),
                n2.until(n2::isDone));
    }

    @Override
    public String name() {
        return "Auton Test2";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, -1, 0), 1),
                new WaypointSE2(new Pose2d(2, 2.5, new Rotation2d(225 * (Math.PI / 180))),
                        new DirectionSE2(1, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t2(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-0.2, 1, 0), 1),
                new WaypointSE2(new Pose2d(3, 5.5, new Rotation2d(270 * (Math.PI / 180))),
                        new DirectionSE2(-1, 1, 0), 1));
        new WaypointSE2(AutonPositions.LEFT_BUMP_PAST,
                new DirectionSE2(1, 0, 1), 1);
        new WaypointSE2(new Pose2d(3, 2.1, new Rotation2d(270 * (Math.PI / 180))),
                new DirectionSE2(1, 0, 1), 1);
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Pose2d start() {
        return new Pose2d(3, 5.5, new Rotation2d(90 * (Math.PI / 180)));
        // Slightly in front of StartingPositions.LEFT_BUMP
        // to make sure the robot starts and ends off of the bump
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1, this::t2);
    }

}
