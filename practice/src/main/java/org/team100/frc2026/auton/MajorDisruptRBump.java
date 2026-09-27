package org.team100.frc2026.auton;

import static org.wpilib.command2.Commands.parallel;

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
import org.team100.lib.trajectory.se2.constraint.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.VelocityLimitRegionConstraint;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class MajorDisruptRBump implements AnnotatedCommand {
    private final LoggerFactory log;
    private final ControllerSE2 controller;
    private final Machinery machinery;
    private final TrajectorySE2Planner planner;

    public MajorDisruptRBump(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            ControllerSE2 controller,
            Machinery machinery) {
        log = parent.name(name());
        this.controller = controller;
        this.machinery = machinery;

        double bumpV = 2; // cartesian velocity over the bump
        List<TimingConstraint> new_constraints = new ArrayList<>(List.of(
                // high velocity, moderate accel
                new ConstantConstraint(8, 20),
                // absolute maxima
                // new SwerveDriveDynamicsConstraint(log, kinodynamics, 1, 1),
                // high yaw limits
                // new YawRateConstraint(log, 8, 20),
                // moderate capsize limits. Note we're not actually concerned about capsize
                // here, we just want to limit tire tread shear
                new CapsizeAccelerationConstraint(8, 20),
                new VelocityLimitRegionConstraint(BumpZones.BLUE_BUMP_LEFT, bumpV),
                new VelocityLimitRegionConstraint(BumpZones.BLUE_BUMP_RIGHT, bumpV),
                new VelocityLimitRegionConstraint(BumpZones.RED_BUMP_LEFT, bumpV),
                new VelocityLimitRegionConstraint(BumpZones.RED_BUMP_RIGHT, bumpV)));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(new_constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    @Override
    public Pose2d start() {
        return StartingPositions.RIGHT_BUMP;
    }

    @Override
    public String name() {
        return "MajorDisruptRBump";
    }

    @Override
    public Command command() {
        DriveWithTrajectoryFunction fn = new DriveWithTrajectoryFunction(
                log,
                machinery.m_drive,
                controller,
                machinery.m_trajectoryViz,
                this::t1);

        return parallel(
                fn
        // // extend when in neutral zone
        // toggle(
        // this::inNeutralZone,
        // machinery.m_intakeExtend.goToExtendedPositionEndlessly(),
        // machinery.m_intakeExtend.goToRetractedPosition()),
        // // roll when extended
        // toggle(
        // this::intakeExtended,
        // parallel( machinery.m_intake.intake(),
        // machinery.m_shooter.shooterFullspeed()),
        // machinery.m_intake.stop())
        );
    }

    @Override
    public List<Function<Pose2d, TrajectorySE2>> trajectoryFns() {
        return List.of(this::t1);
    }

    TrajectorySE2 t1(Pose2d startingPose) {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(startingPose, new DirectionSE2(1, 0, 0), 1),

                new WaypointSE2(new Pose2d(8, 2.5, new Rotation2d(165 * (Math.PI / 180))), new DirectionSE2(1, 0, 0),
                        1),
                new WaypointSE2(new Pose2d(8.1, 7, new Rotation2d(165 * (Math.PI / 180))), new DirectionSE2(1, 0, 0), 1)

        //
        );
        return planner.restToRest(waypoints);
    }
}