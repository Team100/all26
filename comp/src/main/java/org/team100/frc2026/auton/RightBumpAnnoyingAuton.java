package org.team100.frc2026.auton;

import static org.wpilib.command2.Commands.parallel;
import static org.wpilib.command2.Commands.sequence;

import java.util.ArrayList;
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
import org.team100.lib.trajectory.se2.constraint.VelocityLimitRegionConstraint;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** An example of a simple sequence */
public class RightBumpAnnoyingAuton implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final List<TimingConstraint> constraints;
    private final TrajectorySE2Factory trajectoryFactory;
    private final PathSE2Factory pathFactory;
    private final TrajectorySE2Planner planner;

    public RightBumpAnnoyingAuton(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;
        constraints = new TimingConstraintFactory(kinodynamics).auto();
        // In meters/second
        double maxBumpVelocity = 2;
        List<TimingConstraint> new_constraints = new ArrayList<>(constraints);

        // create a new VelocityRegionContstraint `slow_bump_zone`
        VelocityLimitRegionConstraint slow_bump_zone = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone2 = new VelocityLimitRegionConstraint(
                BumpZones.BLUE_BUMP_RIGHT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone3 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_LEFT, maxBumpVelocity);
        VelocityLimitRegionConstraint slow_bump_zone4 = new VelocityLimitRegionConstraint(
                BumpZones.RED_BUMP_RIGHT, maxBumpVelocity);
        new_constraints.add(slow_bump_zone);
        new_constraints.add(slow_bump_zone2);
        new_constraints.add(slow_bump_zone3);
        new_constraints.add(slow_bump_zone4);
        // constraints.add(slow_bump_zone);
        trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public String name() {
        return "Annoying Auton Right Bump";
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(new Pose2d(8.45, 1, new Rotation2d(250 * (Math.PI / 180))),
                        new DirectionSE2(1, 0, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t2(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(new Pose2d(8.45, 4, new Rotation2d(250 * (Math.PI / 180))),
                        new DirectionSE2(0, 1, 0), 1));
        return planner.restToRest(waypoints);
    }

    TrajectorySE2 t3(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose,
                        new DirectionSE2(-1, -1, 0), 1),
                new WaypointSE2(StartingPositions.RIGHT_BUMP,
                        new DirectionSE2(-1, 0, 0), 1),
                new WaypointSE2(AutonPositions.SHOOT_RIGHT,
                        new DirectionSE2(-1, 0, 0), 1));
        return planner.restToRest(waypoints);
    }

    // TrajectorySE2 t4(Pose2d startingPose) {
    // List<WaypointSE2> waypoints = List.of(
    // new WaypointSE2(startingPose,
    // new DirectionSE2(-1, -1, 0), 1),
    // new WaypointSE2(AutonPositions.CLIMB_RIGHT,
    // new DirectionSE2(-1, -1, 0), 1));
    // return planner.restToRest(waypoints);
    // }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction IntakeSetUp = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t1);
        DriveWithTrajectoryFunction IntakeBalls = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t2);
        DriveWithTrajectoryFunction ScoreSetUp = new DriveWithTrajectoryFunction(
                log, machinery.m_drive, controller,
                machinery.m_trajectoryViz, this::t3);
        // DriveWithTrajectoryFunction ClimbSetUp = new DriveWithTrajectoryFunction(
        // log, machinery.m_drive, controller,
        // machinery.m_trajectoryViz, this::t4);

        // Intake, score, climb.
        return sequence(
                parallel(
                        IntakeSetUp.until(IntakeSetUp::isDone).withTimeout(4),
                        // Assumed that the intake shouldn't deploy over the bump
                        // waitSeconds(1).andThen(machinery.m_intakeExtend.goToExtendedPosition())),
                        // waitSeconds(1),

                        parallel(
                                IntakeBalls
                        // machinery.m_intake.intake()).until(IntakeBalls::isDone),
                        // Without telling it to, the intake would only stop spinning
                        // at the end of the auton. Without the timeout, the robot
                        // would not continue the rest of the auton
                        // machinery.m_intake.stop().withTimeout(1),
                        // waitSeconds(1),
                        ),

                        ScoreSetUp.until(ScoreSetUp::isDone)
                // parallel(
                // machinery.m_conveyor.convey(),
                // machinery.m_feeder.proportional(),
                // machinery.m_shooterHood.autoPosition(),
                // machinery.m_shooter.auto()),
                // .withTimeout(1),
                // machinery.m_shooterHood.autoPosition().withTimeout(0.5),
                // machinery.m_shooter.auto().withTimeout(1),
                // waitSeconds(5),
                // machinery.m_shooter.stop().withTimeout(1)
                ));
    }

    // ClimbSetUp.until(ClimbSetUp::isDone));
    // }

    @Override
    public Pose2d start() {
        return StartingPositions.RIGHT_BUMP;
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1, this::t2, this::t3);
    }

}
