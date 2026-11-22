package org.team100.frc2026.auton;

import java.util.List;
import java.util.function.Function;

import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingTransform2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;

/**
 * Demonstrate ~SE(2) (i.e. winding) paths
 * 
 * Makes a wide left turn, while rotating clockwise.
 */
public class TurnLeftWhileSpinningRight implements AnnotatedCommand {
    private final LoggerFactory log;
    private final Machinery machinery;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public TurnLeftWhileSpinningRight(LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.machinery = machinery;
        // Note slow constraints here
        trajectoryFactory = new TrajectorySE2Factory(new TimingConstraintFactory(kinodynamics).slow());
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        command = n1.until(n1::isDone);
    }

    TrajectorySE2 t1(Pose2d p1) {
        // start towards where the robot is facing (robot-relative +x)
        DirectionSE2 d1 = DirectionSE2.fromDirections(p1.getRotation(), 0);
        WindingWaypointSE2 w1 = new WindingWaypointSE2(new WindingPose2d(p1), d1, 1);
        WindingTransform2d t1 = new WindingTransform2d(2, 2, -3 * Math.PI / 2);
        WindingPose2d p2 = new WindingPose2d(p1).plus(t1);
        // end up in the direction we're facing
        DirectionSE2 d2 = DirectionSE2.fromDirections(p2.getRotation(), 0);
        WindingWaypointSE2 w2 = new WindingWaypointSE2(p2, d2, 1);
        List<WindingWaypointSE2> waypoints = List.of(w1, w2);
        return planner.restToRest(waypoints);
    }

    @Override
    public String name() {
        return "Turn Left While Spinning Right";
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Pose2d start() {
        return machinery.m_drive.getState().pose();
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }
}
