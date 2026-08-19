package org.team100.lib.controller.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;
import org.team100.lib.subsystems.se2.MockSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class ReferenceControllerSE2Test implements Timeless {
    private static final boolean DEBUG = false;

    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTrajectoryStart() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        // stepTime();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).point().velocity(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());

        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, t);
        VelocityReferenceControllerSE2 c = new VelocityReferenceControllerSE2(
                logger, drive, controller, reference);

        stepTime();
        c.execute();

        // we don't advance because we're still steering.
        // this next-setpoint is from "preview"
        // and our current setpoint is equal to the measurement.
        stepTime();
        c.execute();
        // assertEquals(0.098, drive.m_setpoint.x(), DELTA);
        // assertEquals(0, drive.m_setpoint.y(), DELTA);
        // assertEquals(0, drive.m_setpoint.theta(), DELTA);

        stepTime();
        c.execute();
        assertEquals(0.139, drive.m_setpoint.x().v(), DELTA);
        assertEquals(0, drive.m_setpoint.y().v(), DELTA);
        assertEquals(0, drive.m_setpoint.theta().v(), DELTA);

        // more normal driving
        stepTime();
        c.execute();
        assertEquals(0.179, drive.m_setpoint.x().v(), DELTA);
        assertEquals(0, drive.m_setpoint.y().v(), DELTA);
        assertEquals(0, drive.m_setpoint.theta().v(), DELTA);

        // etc
        stepTime();
        c.execute();
        assertEquals(0.221, drive.m_setpoint.x().v(), DELTA);
        assertEquals(0, drive.m_setpoint.y().v(), DELTA);
        assertEquals(0, drive.m_setpoint.theta().v(), DELTA);
    }

    @Test
    void testTrajectoryDone() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        stepTime();
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, Rotation2d.kZero),
                new Pose2d(1, 0, Rotation2d.kZero));
        // first state is motionless
        assertEquals(0, t.sample(0).point().velocity(), DELTA);
        ControllerSE2 controller = ControllerFactorySE2.test(logger);

        // initially at rest
        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());

        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, t);
        VelocityReferenceControllerSE2 c = new VelocityReferenceControllerSE2(
                logger, drive, controller, reference);

        // the measurement never changes but that doesn't affect "done" as far as the
        // trajectory is concerned.
        for (int i = 0; i < 200; ++i) {
            stepTime();
            c.execute();
            if (DEBUG)
                System.out.printf("%s\n", drive.m_setpoint);
            // we have magically reached the end (immediately)
            drive.m_state = new ModelSE2(new Pose2d(1, 0, Rotation2d.kZero));
        }
        assertTrue(c.isDone());

    }

    @Test
    void testFieldRelativeTrajectory() {
        List<WaypointSE2> waypoints = new ArrayList<>();
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(),
                        Rotation2d.fromDegrees(180)),
                new DirectionSE2(1, 0, 0), 1));
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(100, 4),
                        Rotation2d.fromDegrees(180)),
                new DirectionSE2(1, 0, 0), 1));
        waypoints.add(new WaypointSE2(
                new Pose2d(
                        new Translation2d(196, 13),
                        Rotation2d.fromDegrees(0)),
                new DirectionSE2(1, 0, 0), 1));

        double start_vel = 0.0;
        double end_vel = 0.0;

        double stepSize = 2;

        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        PathSE2Factory pathFactory = new PathSE2Factory(stepSize, 2, 0.1);
        PathSE2 path = pathFactory.get(splines);

        TrajectorySE2Factory u = new TrajectorySE2Factory(Arrays.asList());
        TrajectorySE2 trajectory = u.fromPath(path, start_vel, end_vel);
        if (DEBUG)
            System.out.printf("TRAJECTORY:\n%s\n", trajectory);

        FullStateControllerSE2 swerveController = new FullStateControllerSE2(
                logger,
                2.4, 2.4,
                0.1, 0.1,
                0.01, 0.02,
                0.01, 0.02);

        MockSubsystemSE2 drive = new MockSubsystemSE2(new ModelSE2());
        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(logger, trajectory);
        VelocityReferenceControllerSE2 referenceController = new VelocityReferenceControllerSE2(
                logger, drive, swerveController, reference);

        Pose2d pose = trajectory.sample(0).point().point().waypoint().pose();
        VelocityControlSE2 velocity = VelocityControlSE2.ZERO;

        double mDt = 0.02;
        int i = 0;
        while (!referenceController.isDone()) {
            if (++i > 500)
                break;
            stepTime();
            drive.m_state = new ModelSE2(pose, velocity.velocity());
            referenceController.execute();
            velocity = drive.m_recentSetpoint;
            // TODO: add acceleration term here
            pose = new Pose2d(
                    pose.getX() + velocity.x().v() * mDt,
                    pose.getY() + velocity.y().v() * mDt,
                    new Rotation2d(pose.getRotation().getRadians() + velocity.theta().v() * mDt));
            if (DEBUG)
                System.out.printf("pose %s vel %s\n", pose, velocity);
        }

        // this should be exactly right but it's not.
        assertEquals(195, pose.getTranslation().getX(), 1);
        assertEquals(13, pose.getTranslation().getY(), 0.4);
        assertEquals(0, pose.getRotation().getRadians(), 0.1);
    }

}
