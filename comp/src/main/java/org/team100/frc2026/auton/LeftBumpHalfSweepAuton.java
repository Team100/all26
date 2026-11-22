package org.team100.frc2026.auton;

import static org.wpilib.command2.Commands.parallel;
import static org.wpilib.command2.Commands.sequence;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.frc2026.robot.Machinery;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
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
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;

/** An example of a simple sequence */
public class LeftBumpHalfSweepAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;
    private final Command command;

    public LeftBumpHalfSweepAuton(
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
                log.name("IntakeSetUp"), machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);

        // Intake, score
        command = sequence(
                parallel(
                        IntakeSetUp.until(IntakeSetUp::isDone).withTimeout(8),
                        // Assumed that the intake shouldn't deploy over the bump
                        sequence(
                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInNeutralZone(machinery.m_drive.getState().translation())),
                                (machinery.m_intakeExtend.goToExtendedPosition()
                                        .andThen(machinery.m_intake.intake())).withTimeout(3),

                                Commands.waitUntil(() -> FieldConstants2026
                                        .isInAllianceZone(machinery.m_drive.getState().translation())),
                                parallel(
                                        machinery.m_intake.stop(),
                                        machinery.m_intakeExtend.goToRetractedPosition(),
                                        machinery.m_shooter.auto()))));
    }

    @Override
    public String name() {
        return "Half Sweep from Left Bump";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WindingWaypointSE2> waypoints = List.of(
                new WindingWaypointSE2(new WindingPose2d(startingPose),
                        new DirectionSE2(1, 0, 0), 1),
                new WindingWaypointSE2(AutonPositions.ABOVE_BALL_FIELD,
                        new DirectionSE2(1, 1, 0), 1),
                new WindingWaypointSE2(AutonPositions.MIDDLE_BALL_FIELD,
                        new DirectionSE2(0, -1, 0), 1),
                new WindingWaypointSE2(new WindingPose2d(6.5, 5.5, 0 ),
                        new DirectionSE2(-1, 0, 0), 1),
                new WindingWaypointSE2(StartingPositions.LEFT_BUMP,
                        new DirectionSE2(-1, 0, 0), 1),
                new WindingWaypointSE2(AutonPositions.SHOOT_LEFT,
                        new DirectionSE2(-1, -1, 0), 1));
        return planner.restToRest(waypoints);
    }

    @Override
    public Command command() {
        return command;
    }

    @Override
    public Pose2d start() {
        return StartingPositions.LEFT_BUMP.pose2d();
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }

}
