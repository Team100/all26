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
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Make a large U shape, ahead and to the left, while rotating
 * 180 degrees. The path is very approximately a half-circle with a radius of
 * about 1 meter.
 */
public class UTurnWithRotation implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public UTurnWithRotation(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        // Note slow constraints here
        constraints = new TimingConstraintFactory(kinodynamics).slow();
        trajectoryFactory = new TrajectorySE2Factory(constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    TrajectorySE2 t1(Pose2d p1) {
        // cartesian is robot-relative +x, also rotate CCW.
        DirectionSE2 d1 = DirectionSE2.fromDirections(p1.getRotation(), 1);
        // the end is the opposite direction.
        DirectionSE2 d2 = DirectionSE2.fromDirections(p1.getRotation().plus(Rotation2d.kPi), 1);
        // making a big U means using a large scale factor
        WaypointSE2 w1 = new WaypointSE2(p1, d1, 2.5);
        // end point is to the left
        Transform2d t1 = new Transform2d(0, 2, Rotation2d.kPi);
        Pose2d p2 = p1.plus(t1);
        // making a big U means using a large scale factor
        WaypointSE2 w2 = new WaypointSE2(p2, d2, 2.5);
        List<WaypointSE2> waypoints = List.of(w1, w2);
        return planner.restToRest(waypoints);
    }

    @Override
    public String name() {
        return "U-Turn With Rotation";
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction n1 = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        return n1.until(n1::isDone);
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
