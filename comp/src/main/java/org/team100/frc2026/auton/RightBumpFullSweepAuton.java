package org.team100.frc2026.auton;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.List;
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
import edu.wpi.first.wpilibj2.command.Commands;

/** An example of a simple sequence */
public class RightBumpFullSweepAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public RightBumpFullSweepAuton(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        // In meters/second
        double maxBumpVelocity = 2;
        List<TimingConstraint> new_constraints = Stream.concat(
                new TimingConstraintFactory(kinodynamics).auto().stream(),
                BumpZones.constraint(maxBumpVelocity).stream()).toList();
        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        DriveWithTrajectoryFunction IntakeSetUp = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);

        // Intake, score
        command = sequence(
                parallel(
                        IntakeSetUp.until(IntakeSetUp::isDone).withTimeout(8),
                        sequence(
                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInNeutralZone(machinery.m_drive.getState().translation())),
                                (machinery.m_intakeExtend.goToExtendedPosition()
                                        .andThen(machinery.m_intake.intake())).withTimeout(4),

                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInAllianceZone(machinery.m_drive.getState().translation())),
                                parallel(
                                        machinery.m_intake.stop(),
                                        machinery.m_intakeExtend.goToRetractedPosition(),
                                        machinery.m_shooter.auto()))));
    }

    @Override
    public String name() {
        return "Full Sweep from Right Bump";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(new Pose2d(7.75, 2, Rotation2d.kCW_90deg),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(new Pose2d(7.75, 6.5, Rotation2d.kCW_90deg),
                        new DirectionSE2(-0.2, 1, 0), 1),
                new WaypointSE2(new Pose2d(7, 2, new Rotation2d(-270 * (Math.PI / 180))),
                        new DirectionSE2(0, -1, 0), 1),
                new WaypointSE2(new Pose2d(4.66, 2.5, new Rotation2d(0 * (Math.PI / 180))),
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.SHOOT_RIGHT,
                        new DirectionSE2(0, 1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Pose2d start() {
        return StartingPositions.RIGHT_BUMP;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }

}
