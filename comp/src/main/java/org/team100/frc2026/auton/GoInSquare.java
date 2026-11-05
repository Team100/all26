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
 * Move in a square, two meters by two meters. 
 * Ends in the same position and facing the 
 * same way as it's starting position.
 */
public class GoInSquare implements AnnotatedCommand {
    private final LoggerFactory log;
    private final Machinery machinery;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public GoInSquare(
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
        // move towards where the robot is facing (robot-relative +x)
        DirectionSE2 d1 = DirectionSE2.irrotational(p1.getRotation());
        WaypointSE2 w1 = new WaypointSE2(p1, d1, 1);
        
        Transform2d t1 = new Transform2d(1, 0, Rotation2d.kZero);
        Pose2d p2 = p1.plus(t1);
        WaypointSE2 w2 = new WaypointSE2(p2, d1, 1);

        Transform2d t2 = new Transform2d(1, 0, new Rotation2d(90 * (Math.PI / 180)));
        DirectionSE2 d3 = DirectionSE2.irrotational(p2.getRotation().getRadians() + (Math.PI / 2));
        Pose2d p3 = p2.plus(t2);
        WaypointSE2 w3 = new WaypointSE2(p3, d3, 1);

        Transform2d t3 = new Transform2d(1, 0, Rotation2d.kZero);
        Pose2d p4 = p3.plus(t3);
        WaypointSE2 w4 = new WaypointSE2(p4, d3, 1);

        Transform2d t4 = new Transform2d(1, 0, new Rotation2d(90 * (Math.PI / 180)));
        DirectionSE2 d5 = DirectionSE2.irrotational(p3.getRotation().getRadians() + (Math.PI / 2));
        Pose2d p5 = p4.plus(t4);
        WaypointSE2 w5 = new WaypointSE2(p5, d5, 1);

        Transform2d t5 = new Transform2d(1, 0, Rotation2d.kZero);
        Pose2d p6 = p5.plus(t5);
        WaypointSE2 w6 = new WaypointSE2(p6, d5, 1);

        Transform2d t6 = new Transform2d(1, 0, new Rotation2d((90 * (Math.PI / 180))));
        DirectionSE2 d7 = DirectionSE2.irrotational(p6.getRotation().getRadians() + (Math.PI / 2));
        Pose2d p7 = p6.plus(t6);
        WaypointSE2 w7 = new WaypointSE2(p7, d7, 1);

        Transform2d t7 = new Transform2d(1, 0, Rotation2d.kZero);
        Pose2d p8 = p7.plus(t7);
        WaypointSE2 w8 = new WaypointSE2(p8, d7, 1);

        Transform2d t8 = new Transform2d(1, 0, new Rotation2d((90 * (Math.PI / 180))));
        DirectionSE2 d9 = DirectionSE2.irrotational(p6.getRotation().getRadians() + (Math.PI / 2));
        Pose2d p9 = p8.plus(t8);
        WaypointSE2 w9 = new WaypointSE2(p9, d9, 1);

        List<WaypointSE2> waypoints = List.of(w1, w2, w3, w4, w5, w6, w7, w8, w9);
        return planner.restToRest(waypoints);
    }

    @Override
    public String name() {
        return "Go in square";
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
