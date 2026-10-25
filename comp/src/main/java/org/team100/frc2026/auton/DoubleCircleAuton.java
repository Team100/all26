package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.stream.Stream;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.se2.commands.DriveWithTrajectoryFunctionWithOverride;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.targeting.Solver;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.SwerveDriveDynamicsConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Full speed sweep, shoot-on-the-move, repeat */
public class DoubleCircleAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final Machinery machinery;
    private final TrajectorySE2Planner planner;
    private final Solver m_solver;
    private final Command command;

    public DoubleCircleAuton(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Solver solver,
            Machinery machinery) {
        log = parent.name(name());
        m_solver = solver;
        this.machinery = machinery;

        double bumpV = 2; // cartesian velocity over the bump
        List<TimingConstraint> new_constraints = Stream.concat(
                BumpZones.constraint(bumpV).stream(),
                List.of(
                        // high velocity, moderate accel
                        new ConstantConstraint(5, 20),
                        // absolute maxima
                        new SwerveDriveDynamicsConstraint(kinodynamics, 1, 1),
                        // high yaw limits
                        new YawRateConstraint(10, 20),
                        // moderate capsize limits. Note we're not actually concerned about capsize
                        // here, we just want to limit tire tread shear
                        new CapsizeAccelerationConstraint(5, 20)).stream())
                .toList();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        DriveWithTrajectoryFunctionWithOverride navigate = new DriveWithTrajectoryFunctionWithOverride(
                log,
                machinery.m_drive,
                controller,
                machinery.m_trajectoryViz,
                this::t1, m_solver,
                this::inAllianceZone);
        Command extendWhenInNeutralZone = toggle(
                this::inNeutralZone,
                machinery.m_intakeExtend.goToExtendedPositionEndlessly(),
                machinery.m_intakeExtend.goToRetractedPosition());
        Command rollWhenExtended = toggle(
                this::intakeExtended,
                machinery.m_intake.intake(),
                machinery.m_intake.stop());
        // this doesn't intake because of the parallel requirement issue.
        Command shootWhenInAllianceZone = toggle(
                this::inAllianceZone,
                parallel(
                        // machinery.m_intake.intake(),
                        machinery.m_shooter.auto()),
                parallel(
                        // machinery.m_intake.intake(),
                        machinery.m_shooter.stop()));
        command = parallel(
                navigate,
                extendWhenInNeutralZone,
                rollWhenExtended,
                shootWhenInAllianceZone);
    }

    @Override
    public String name() {
        return "Double Circle";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                // bump
                new WaypointSE2(startingPose, new DirectionSE2(1, -0.5, 0), 1),
                // sweep lead-in
                new WaypointSE2(new Pose2d(7.75, 7.5, new Rotation2d(1.57)), new DirectionSE2(1, 0, 0), 1),
                // sweep over the center
                new WaypointSE2(new Pose2d(8.25, 6.5, new Rotation2d(1.57)), new DirectionSE2(0, -1, 0), 1),
                new WaypointSE2(new Pose2d(8.25, 1.5, new Rotation2d(1.57)), new DirectionSE2(0, -1, 0), 1),
                // sweep lead-out
                new WaypointSE2(new Pose2d(7.75, 0.5, new Rotation2d(1.57)), new DirectionSE2(-1, 0, 0), 1),
                // bump
                new WaypointSE2(new Pose2d(4.5, 2.25, new Rotation2d(1.57)), new DirectionSE2(-1, -0.5, 0), 1),
                // shoot with lag
                // note the closest approach is pretty close, because we're going
                // pretty fast
                new WaypointSE2(new Pose2d(3, 2.25, new Rotation2d(1.1)), new DirectionSE2(-1, 1, 0), 1),
                new WaypointSE2(new Pose2d(2.25, 4, new Rotation2d(-0.2)), new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(new Pose2d(3, 5.75, new Rotation2d(-1.4)), new DirectionSE2(1, 1, 0), 1),
                // bump
                new WaypointSE2(new Pose2d(4.5, 5.75, new Rotation2d(0)), new DirectionSE2(1, -0.5, 0), 1),
                // sweep lead-in
                new WaypointSE2(new Pose2d(7, 7.5, new Rotation2d(1.57)), new DirectionSE2(1, 0, 0), 1),
                // sweep closer to our side
                new WaypointSE2(new Pose2d(7.5, 6.5, new Rotation2d(1.57)), new DirectionSE2(0, -1, 0), 1),
                new WaypointSE2(new Pose2d(7.5, 1.5, new Rotation2d(1.57)), new DirectionSE2(0, -1, 0), 1),
                // sweep lead-out
                new WaypointSE2(new Pose2d(7, 0.5, new Rotation2d(1.57)), new DirectionSE2(-1, 0, 0), 1),
                // bump
                new WaypointSE2(new Pose2d(4.5, 2.25, new Rotation2d(1.57)), new DirectionSE2(-1, -0.5, 0), 1),
                // shoot
                new WaypointSE2(new Pose2d(3, 2.25, new Rotation2d(1.1)), new DirectionSE2(-1, 1, 0), 1),
                new WaypointSE2(new Pose2d(2.25, 4, new Rotation2d(-0.2)), new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(new Pose2d(3, 5.75, new Rotation2d(-1.4)), new DirectionSE2(1, 1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }

    /** Runs the commands according to the condition, interrupting to transition. */
    private Command toggle(BooleanSupplier condition, Command whenTrue, Command whenFalse) {
        return repeatingSequence(whenFalse.until(condition), whenTrue.onlyWhile(condition));
    }

    private boolean inAllianceZone() {
        return FieldConstants2026.isInAllianceZone(machinery.m_drive.getState().translation());
    }

    private boolean intakeExtended() {
        return machinery.m_intakeExtend.isOut();
    }

    private boolean inNeutralZone() {
        return FieldConstants2026.isInNeutralZone(machinery.m_drive.getState().translation());
    }
}
