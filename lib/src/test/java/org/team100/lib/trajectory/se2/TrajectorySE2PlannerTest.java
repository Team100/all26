package org.team100.lib.trajectory.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se2.constraint.CapsizeAccelerationConstraint;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.SwerveDriveDynamicsConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraintFactory;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;
import org.team100.lib.trajectory.se2.examples.TrajectoryExamples;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class TrajectorySE2PlannerTest implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;

    @Test
    void testLinear() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectorySE2 t = planner.restToRest(waypoints);
        assertEquals(17, t.length());
        TrajectorySE2Entry tp = t.getPoint(0);
        // start at zero velocity
        assertEquals(0, tp.point().velocity(), DELTA);
        TrajectorySE2Entry p = t.getPoint(8);
        assertEquals(0.5, p.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.point().point().waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testLinearRealistic() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2));
        // these are the same as StraightLineTrajectoryTest.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(1, 1, limits),
                new SwerveDriveDynamicsConstraint(limits, 1, 1),
                new YawRateConstraint(limits, 0.2),
                new CapsizeAccelerationConstraint(limits, 0.2));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectorySE2 t = planner.restToRest(waypoints);
        assertEquals(17, t.length());
        TrajectorySE2Entry tp = t.getPoint(0);
        assertEquals(0, tp.point().velocity(), DELTA);
        TrajectorySE2Entry p = t.getPoint(8);
        assertEquals(0.5, p.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.point().point().waypoint().course().headingRate(), DELTA);
    }

    /**
     * 0.23 ms on my machine.
     * 
     * See PathFactoryTest::testPerformance().
     */
    @Test
    void testPerformance() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(),
                                new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 1),
                                new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1.2));
        List<TimingConstraint> constraints = new ArrayList<>();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        long startTimeNs = System.nanoTime();
        TrajectorySE2 t = new TrajectorySE2();
        // for profiling
        // final long iterations = 10000000000l;
        final long iterations = 100l;
        for (long i = 0; i < iterations; ++i) {
            t = planner.restToRest(waypoints);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(33, t.length());
        TrajectorySE2Entry p = t.getPoint(12);
        assertEquals(0.605, p.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.point().point().waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testRestToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(0, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        TrajectoryExamples ex = new TrajectoryExamples(planner);
        TrajectorySE2 trajectory = ex.restToRest(start.pose(), end);
        assertEquals(1.565, trajectory.duration(), DELTA);

        /** progress along trajectory */
        double m_timeS = 0;

        // initial velocity is zero.
        assertEquals(0, trajectory.sample(m_timeS).point().velocity(), DELTA);

        double maxDriveVelocityM_S = swerveKinodynamics.getMaxDriveVelocityM_S();
        double maxDriveAccelerationM_S2 = swerveKinodynamics.getMaxDriveAccelerationM_S2();
        assertEquals(5, maxDriveVelocityM_S);
        assertEquals(10, maxDriveAccelerationM_S2);
        for (TrajectorySE2Entry p : trajectory.getPoints()) {
            assertTrue(p.point().velocity() - 0.001 <= maxDriveVelocityM_S,
                    String.format("%f %f", p.point().velocity(), maxDriveVelocityM_S));
            assertTrue(p.point().accel() - 0.001 <= maxDriveAccelerationM_S2,
                    String.format("%f %f", p.point().accel(), maxDriveAccelerationM_S2));
        }
    }

    @Test
    void testMovingToRest() {
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(1, 0, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);

        VelocitySE2 startVelocity = start.velocity();

        Translation2d full = end.getTranslation().minus(start.translation());
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        TrajectorySE2 traj = planner.generateTrajectory(
                List.of(
                        new WaypointSE2(
                                start.pose(),
                                DirectionSE2.irrotational(startingAngle),
                                e1),
                        new WaypointSE2(
                                end,
                                DirectionSE2.irrotational(courseToGoal),
                                1.2)),
                startVelocity.norm(),
                0);
        assertEquals(1.176, traj.duration(), DELTA);
    }

    /**
     * This is a curve that goes +y, turns sharply towards +x, with a more gradual
     * curve after that.
     * 
     * Initial velocity is clamped (with a warning)
     * Max braking to the apex
     * Coast through the apex
     * Max accel for about half of the rest
     * Max decel to the end
     */
    @Test
    void test2d() {
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(1, 1),
                new CapsizeAccelerationConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        ModelSE2 start = new ModelSE2(Pose2d.kZero, new VelocitySE2(0, 1, 0));
        Pose2d end = new Pose2d(1, 0, Rotation2d.kZero);
        VelocitySE2 startVelocity = start.velocity();

        Translation2d full = end.getTranslation().minus(start.translation());
        Rotation2d courseToGoal = full.getAngle();
        Rotation2d startingAngle = startVelocity.angle().orElse(courseToGoal);

        // use the start velocity to adjust the first magic number.
        // divide by the distance because the spline multiplies by it
        double e1 = 2.0 * startVelocity.norm() / full.getNorm();
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(start.pose(), DirectionSE2.irrotational(startingAngle), e1),
                new WaypointSE2(end, DirectionSE2.irrotational(courseToGoal), 1.2));
        TrajectorySE2 traj = planner.generateTrajectory(
                waypoints, startVelocity.norm(), 0);
        if (DEBUG)
            traj.dump();
        assertEquals(2.757, traj.duration(), DELTA);
    }

    /**
     * Straight for 1m, approximately a quarter-circle
     * 
     * Constant velocity on the straight
     * Maximum braking just before the start of the curve
     * Declining braking as the curvature grows
     * No braking at all at the apex
     * Gradual acceleration exiting the curve
     * Maximum acceleration at the exit, to the max V
     * Constant velocity on the straight
     * 
     * These curves should be symmetric around the apex
     */
    @Test
    void test2d2() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(1, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 1, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3),
                new WaypointSE2(new Pose2d(2, 2, new Rotation2d()), new DirectionSE2(0, 1, 0), 1.3));

        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(1, 1),
                new CapsizeAccelerationConstraint(0.5, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);

        TrajectorySE2 traj = planner.generateTrajectory(waypoints, 1, 1);
        if (DEBUG)
            traj.dump();
        assertEquals(4.603, traj.duration(), DELTA);
    }

    /**
     * accelerating at the start
     * slowing before the "slow zone" in the middle
     * speeding up again
     * slowing to a stop at the end
     */
    @Test
    void testVariableConstraint() {

        class ConditionalTimingConstraint implements TimingConstraint {
            @Override
            public double maxV(PathSE2Point point) {
                double x = point.waypoint().pose().getTranslation().getX();
                if (x < 1.5) {
                    return 2.0;
                }
                if (x < 2.5) {
                    return 1.0;
                }
                return 2.0;

            }

            @Override
            public double maxAccel(PathSE2Point point, double velocity) {
                return 2;
            }

            @Override
            public double maxDecel(PathSE2Point point, double velocity) {
                return -1;
            }
        }

        List<TimingConstraint> constraints = List.of(
                new ConditionalTimingConstraint());
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(new Pose2d(0, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3),
                new WaypointSE2(new Pose2d(4, 0, new Rotation2d()), new DirectionSE2(1, 0, 0), 1.3));
        TrajectorySE2 traj = planner.generateTrajectory(waypoints, 0, 0);
        if (DEBUG)
            traj.dump();

    }

    /**
     * Yields a straight line.
     * 
     * TrajectoryPlanner.restToRest() has several overloads: the one that takes
     * two non-holonomic poses draws a straight line between them.
     */
    @Test
    void testSimple() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 0.1),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(p);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(10, 1, new Rotation2d()));

        TrajectorySE2ToVectorSeries converter = new TrajectorySE2ToVectorSeries(0.5);
        List<VectorSeries> series = converter.convert(t);
        ChartUtil.plotOverlay(series, 100);
    }

    /** Turning in place does not work */
    @Test
    void testTurnInPlace() throws InterruptedException {
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(List.of(new ConstantConstraint(1, 0.1)));
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectoryExamples ex = new TrajectoryExamples(p);
        TrajectorySE2 t = ex.restToRest(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(0, 0, new Rotation2d(1)));
        assertTrue(t.isEmpty());
    }

    @Test
    void testCircle() {
        // see HolonomicSplineSE2Test.testCircle();
        // this is to see how to create the dtheta and curvature
        // without the spline.
        double scale = 1.3;
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(-1, 0), Rotation2d.kZero),
                new DirectionSE2(0, -1, 1), scale);
        WaypointSE2 p3 = new WaypointSE2(
                new Pose2d(new Translation2d(0, -1), Rotation2d.kCCW_90deg),
                new DirectionSE2(1, 0, 1), scale);

        List<WaypointSE2> waypoints = List.of(p0, p1, p2, p3, p0);

        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectorySE2 trajectory = planner.generateTrajectory(waypoints, 0, 0);

        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.25).convert(trajectory);
        ChartUtil.plotOverlay(series, 300);
    }

    @Test
    void testDheading() {
        double scale = 1.3;
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), Rotation2d.k180deg),
                new DirectionSE2(0, 1, 1), scale);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kCW_90deg),
                new DirectionSE2(-1, 0, 1), scale);
        List<WaypointSE2> waypoints = List.of(w0, w1);

        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        TrajectorySE2 trajectory = planner.generateTrajectory(waypoints, 0, 0);
        double duration = trajectory.duration();
        TrajectorySE2Entry p0 = trajectory.sample(0);
        if (DEBUG)
            System.out.println(
                    "t, intrinsic_heading_dt, heading_dt, intrinsic_ca, extrinsic_ca, extrinsic v, intrinsic v, dcourse, dcourse1");
        for (double t = 0.04; t < duration; t += 0.04) {
            TrajectorySE2Entry p1 = trajectory.sample(t);
            Rotation2d heading0 = p0.point().point().waypoint().pose().getRotation();
            Rotation2d heading1 = p1.point().point().waypoint().pose().getRotation();
            double dheading = heading1.minus(heading0).getRadians();
            // compute time derivative of heading two ways:
            // this just compares the poses and uses the known time step
            double dheadingDt = dheading / 0.04;
            // this uses the intrinsic heading rate and the velocity
            // rad/m * m/s = rad/s
            double intrinsicDheadingDt = p0.point().point().waypoint().course().headingRate() * p0.point().velocity();
            // curvature is used to compute centripetal acceleration
            // ca = v^2*curvature
            DirectionSE2 course0 = p0.point().point().waypoint().course();
            DirectionSE2 course1 = p1.point().point().waypoint().course();
            p1.point().point().waypoint().pose().log(p0.point().point().waypoint().pose());
            double dcourse1 = Metrics.translationalNorm(course1.minus(course0));
            double dcourse = course1.toRotation().minus(course0.toRotation()).getRadians();
            double intrinsicCa = p0.point().velocity() * p0.point().velocity() * p0.point().point().k();

            if (DEBUG)
                System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                        t, intrinsicDheadingDt, dheadingDt,
                        intrinsicCa, p0.point().velocity(),
                        dcourse, dcourse1);
            p0 = p1;
        }

        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.25).convert(trajectory);
        ChartUtil.plotOverlay(series, 500);
    }

    /**
     * Yields a curve.
     * 
     * A WaypointSE2 allows separate specification of heading (which way the
     * front of the robot is facing) and course (which way the robot is moving).
     * 
     * In this case, is facing +x, and moving +x, and it ends up moving +y but
     * facing the other way (i.e. backwards)
     */
    @Test
    void testCurved() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(new Translation2d(9, 9), new Rotation2d(-Math.PI / 2)),
                        new DirectionSE2(0, 1, 0), 1));
        TrajectorySE2 t = p.restToRest(waypoints);

        TrajectorySE2ToVectorSeries converter = new TrajectorySE2ToVectorSeries(0.5);
        List<VectorSeries> series = converter.convert(t);
        ChartUtil.plotOverlay(series, 100);
    }

    /**
     * You can specify interior waypoints as well as start and end points. Note that
     * specifying many such points will make the curve harder to calculate and
     * harder to make smooth.
     */
    @Test
    void testMultipleWaypoints() throws InterruptedException {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(new Translation2d(5, 5), new Rotation2d(-2)),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(new Translation2d(9, 9), new Rotation2d(-Math.PI / 2)),
                        new DirectionSE2(0, 1, 0), 1));
        TrajectorySE2 t = p.restToRest(waypoints);

        TrajectorySE2ToVectorSeries converter = new TrajectorySE2ToVectorSeries(0.3);

        List<VectorSeries> series = converter.convert(t);
        ChartUtil.plotOverlay(series, 100);
    }

}
