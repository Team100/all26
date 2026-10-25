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
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.subsystems.se2.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunction;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.smartdashboard.SmartDashboard;

public class IntakeOutake implements AnnotatedCommand {
    private final LoggerFactory m_log;
    private final Machinery m_machinery;
    private final ControllerSE2 m_controller;
    private final TrajectorySE2Planner m_planner;
    private final PathSE2Factory m_pathFactory;
    private final TrajectorySE2Factory m_trajectoryFactory;

    public IntakeOutake(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        List<TimingConstraint> constraints = new TimingConstraintFactory(kinodynamics).auto();
        this.m_pathFactory = new PathSE2Factory();
        this.m_trajectoryFactory = new TrajectorySE2Factory(constraints);
        this.m_planner = new TrajectorySE2Planner(m_pathFactory, m_trajectoryFactory);
        this.m_log = parent.name(name());
        this.m_machinery = machinery;
        this.m_controller = controller;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::drivingForward);
    }

    public Command command() {
        DriveWithTrajectoryFunction driveCommand = new DriveWithTrajectoryFunction(
                m_log, m_machinery.m_drive, m_controller,
                m_machinery.m_trajectoryViz, this::drivingForward);
        DriveWithTrajectoryFunction driveCommandAfter = new DriveWithTrajectoryFunction(
                m_log, m_machinery.m_drive, m_controller,
                m_machinery.m_trajectoryViz, this::drivingForwardAfter);
        DriveWithTrajectoryFunction driveBack = new DriveWithTrajectoryFunction(
                m_log, m_machinery.m_drive, m_controller,
                m_machinery.m_trajectoryViz, this::drivingBack);
        HolonomicProfile turnProfile = HolonomicProfileFactory.get(
                m_machinery.m_swerveKinodynamics, 0.5, 0.5, 0.5, 0.5);
        DriveToPoseWithProfile turnCommand = new DriveToPoseWithProfile(
                m_log, m_machinery.m_drive, m_controller, turnProfile,
                this::leftTurnGoal);
        DriveToPoseWithProfile turnBackCommand = new DriveToPoseWithProfile(
                m_log, m_machinery.m_drive, m_controller, turnProfile,
                this::leftOriginalGoal);
        return Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Driving"))
                .andThen(driveCommand.until(driveCommand::isDone))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Turning Left")))
                .andThen(turnCommand.until(turnCommand::isDone).withTimeout(3))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Driving Forward After")))
                .andThen(driveCommandAfter.until(driveCommandAfter::isDone))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Intake")))
                .andThen(m_machinery.m_subsystem.velocity(1).withTimeout(3))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Turning Back")))
                .andThen(turnBackCommand.until(turnBackCommand::isDone).withTimeout(3))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Driving Back")))
                .andThen(driveBack.until(driveBack::isDone))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Auton Step", "Finished")));
    }

    public String name() {
        return "Drive Intake Back";
    }

    @Override
    public Pose2d start() {
        return new Pose2d(3.5, 0.7, Rotation2d.kZero);
    }

    TrajectorySE2 drivingForward(Pose2d startingPose) {
        DirectionSE2 course = DirectionSE2.irrotational(startingPose.getRotation());
        WaypointSE2 start = new WaypointSE2(startingPose, course, 1);
        Transform2d forwardFourMeters = new Transform2d(4, 0, Rotation2d.kZero);
        Pose2d endingPose = startingPose.plus(forwardFourMeters);
        WaypointSE2 end = new WaypointSE2(endingPose, course, 1);
        List<WaypointSE2> waypoints = List.of(start, end);
        return m_planner.restToRest(waypoints);
    }

    Pose2d leftTurnGoal() {
        Pose2d currentPose = m_machinery.m_drive.getState().pose();
        Rotation2d targetRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(90));
        return new Pose2d(currentPose.getTranslation(), targetRotation);
    }

    TrajectorySE2 drivingForwardAfter(Pose2d startingPose) {
        DirectionSE2 course = DirectionSE2.irrotational(startingPose.getRotation());
        WaypointSE2 start = new WaypointSE2(startingPose, course, 1);
        Transform2d forwardTwoMeters = new Transform2d(2, 0, Rotation2d.kZero);
        Pose2d endingPose = startingPose.plus(forwardTwoMeters);
        WaypointSE2 end = new WaypointSE2(endingPose, course, 1);
        List<WaypointSE2> waypoints = List.of(start, end);
        return m_planner.restToRest(waypoints);
    }

    Pose2d leftOriginalGoal() {
        Pose2d currentPose = m_machinery.m_drive.getState().pose();
        Rotation2d targetRotation = currentPose.getRotation().minus(Rotation2d.fromDegrees(180));
        return new Pose2d(currentPose.getTranslation(), targetRotation);
    }

    TrajectorySE2 drivingBack(Pose2d startingPose) {
        DirectionSE2 course = DirectionSE2.irrotational(startingPose.getRotation());
        WaypointSE2 start = new WaypointSE2(startingPose, course, 1);
        Transform2d forwardFourMeters = new Transform2d(2, 0, Rotation2d.kZero);
        Pose2d endingPose = startingPose.plus(forwardFourMeters);
        WaypointSE2 end = new WaypointSE2(endingPose, course, 1);
        List<WaypointSE2> waypoints = List.of(start, end);
        return m_planner.restToRest(waypoints);
    }
}