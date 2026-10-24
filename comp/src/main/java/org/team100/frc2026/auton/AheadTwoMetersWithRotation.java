package org.team100.frc2026.auton;

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
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move two meters in whatever direction the robot is facing,
 * while rotating 180 degrees.
 */
public class AheadTwoMetersWithRotation implements AnnotatedCommand {
    private final LoggerFactory log;
    private final Machinery machinery;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public AheadTwoMetersWithRotation(
            LoggerFactory parent,
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
        // cartesian is robot-relative +x, also rotate CCW.
        DirectionSE2 d1 = DirectionSE2.fromDirections(p1.getRotation(), 1);
        WaypointSE2 w1 = new WaypointSE2(p1, d1, 1);
        Transform2d t1 = new Transform2d(2, 0, Rotation2d.kPi);
        Pose2d p2 = p1.plus(t1);
        WaypointSE2 w2 = new WaypointSE2(p2, d1, 1);
        List<WaypointSE2> waypoints = List.of(w1, w2);
        return planner.restToRest(waypoints);
    }

    @Override
    public String name() {
        return "Ahead Two Meters With Rotation";
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
