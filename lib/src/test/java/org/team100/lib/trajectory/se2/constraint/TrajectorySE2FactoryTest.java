package org.team100.lib.trajectory.se2.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.path.se2.PathSE2ToVectorSeries;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectorySE2FactoryTest {
    private static final boolean DEBUG = false;
    public static final double EPSILON = 1e-12;
    private static final double DELTA = 0.01;

    /** Straight, then s-shaped, then straight. **/
    public static final List<WaypointSE2> waypoints = Arrays.asList(
            WaypointSE2.irrotational(new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
            WaypointSE2.irrotational(new Pose2d(2, 0, new Rotation2d(0)), 0, 1.2),
            WaypointSE2.irrotational(new Pose2d(3, 1, new Rotation2d(0)), 0, 1.2),
            WaypointSE2.irrotational(new Pose2d(5, 1, new Rotation2d(0)), 0, 1.2));

    /** Low accel, high velocity, makes a triangle profile */
    @Test
    void testConstrained1() {
        double maxV = 20;
        double maxA = 5;
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(maxV, maxA));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        double start_vel = 0;
        double end_vel = 0;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        if (DEBUG)
            trajectory.dump();
        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.1).convert(trajectory);
        List<VectorSeries> series2 = new TrajectorySE2ToVectorSeries(0.005).accel(trajectory);
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 200);
        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);
        verifyDecelConstraints(trajectory, constraints);

        verifyAccel(trajectory);
    }

    /** Low accel, low velocity, makes a trapezoid profile */
    @Test
    void testConstrained2() {
        double maxV = 3;
        double maxA = 5;
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(maxV, maxA));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        double start_vel = 0;
        double end_vel = 0;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        if (DEBUG)
            trajectory.dump();
        ChartUtil.plotOverlay(new TrajectorySE2ToVectorSeries(0.1).convert(trajectory), 200);
        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);

        verifyDecelConstraints(trajectory, constraints);
        verifyAccel(trajectory);
    }

    /** Initial and terminal velocities makes a kind of trapezoid. */
    @Test
    void testConstrained3() {
        double maxV = 6;
        double maxA = 5;
        List<TimingConstraint> constraints = List.of(new ConstantConstraint(maxV, maxA));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        double start_vel = 5;
        double end_vel = 2;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        if (DEBUG)
            trajectory.dump();
        ChartUtil.plotOverlay(new TrajectorySE2ToVectorSeries(0.1).convert(trajectory), 200);

        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);
        verifyDecelConstraints(trajectory, constraints);
        verifyAccel(trajectory);

    }

    /**
     */
    @Test
    void testCentripetalConstraint1() {
        List<TimingConstraint> constraints = List.of(
                new CapsizeAccelerationConstraint(2, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        double start_vel = 0;
        double end_vel = 0;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        if (DEBUG)
            trajectory.dump();
        ChartUtil.plotOverlay(new TrajectorySE2ToVectorSeries(0.1).convert(trajectory), 200);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);
        verifyDecelConstraints(trajectory, constraints);
        verifyAccel(trajectory);
    }

    @Test
    void testCentripetalConstraint2() {
        List<TimingConstraint> constraints = List.of(
                new CapsizeAccelerationConstraint(3, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        double start_vel = 0;
        double end_vel = 0;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        if (DEBUG)
            trajectory.dump();
        ChartUtil.plotOverlay(new TrajectorySE2ToVectorSeries(0.1).convert(trajectory), 200);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);
        verifyDecelConstraints(trajectory, constraints);
        verifyAccel(trajectory);
    }

    @Test
    void testCentripetalConstraint3() {
        List<TimingConstraint> constraints = List.of(
                new CapsizeAccelerationConstraint(2, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        PathSE2 path = getPath();
        // higher than this gets clamped, so don't do that.
        double start_vel = 3;
        double end_vel = 2;
        TrajectorySE2 trajectory = trajectoryFactory.fromPath(path, start_vel, end_vel);
        if (DEBUG)
            trajectory.dump();
        ChartUtil.plotOverlay(new TrajectorySE2ToVectorSeries(0.1).convert(trajectory), 200);
        assertEquals(55, trajectory.length());
        assertEquals(start_vel, trajectory.sample(0).point().velocity(), EPSILON);
        assertEquals(end_vel, trajectory.getLastPoint().point().velocity(), EPSILON);
        verifyVelocityConstraints(trajectory, constraints);
        verifyAccelConstraints(trajectory, constraints);
        verifyDecelConstraints(trajectory, constraints);
        verifyAccel(trajectory);
    }

    /**
     * 0.7 ms on my machine. Could be faster.
     * 
     * See PathFactoryTest::testPerformance()
     */
    @Test
    void testPerformance() {
        // A sweeping left turn without rotation.
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(new Translation2d(), new Rotation2d()),
                        new DirectionSE2(1, 0, 0), 1.2),
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                        new DirectionSE2(0, 1, 0), 1.2));
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.05, 0.2);

        TrajectorySE2 trajectory = new TrajectorySE2();
        TrajectorySE2Factory m_trajectoryFactory = new TrajectorySE2Factory(new ArrayList<>());

        long startTimeNs = System.nanoTime();
        int N = 100;
        for (int i = 0; i < N; ++i) {
            // this takes almost all the time
            PathSE2 path = pathFactory.get(splines);
            // this takes very little time
            trajectory = m_trajectoryFactory.fromPath(path, 0, 0);
        }
        long endTimeNs = System.nanoTime();

        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / N);
        }
        assertEquals(33, trajectory.length());
        TrajectorySE2Entry p = trajectory.getPoint(12);
        assertEquals(0.605, p.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.point().point().waypoint().course().headingRate(), DELTA);
    }

    /**
     * Produce the path for testing.
     */
    private PathSE2 getPath() {
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        PathSE2Factory pathFactory = new PathSE2Factory(0.2, 0.2, 0.2);
        PathSE2 path = pathFactory.get(splines);
        List<VectorSeries> series = new PathSE2ToVectorSeries(0.1).convert(path);
        ChartUtil.plotOverlay(series, 200);
        assertEquals(55, path.length());
        return path;
    }

    /** Verify velocity constraints. */
    private void verifyVelocityConstraints(TrajectorySE2 trajectory, List<TimingConstraint> constraints) {
        double margin = 1e-12;
        for (int i = 0; i < trajectory.length(); ++i) {
            TrajectorySE2Entry state = trajectory.getPoint(i);
            for (TimingConstraint constraint : constraints) {
                String name = constraint.getClass().getSimpleName();
                double stateV = state.point().velocity();
                double maxV = constraint.maxV(state.point().point());
                assertTrue(stateV - margin <= maxV,
                        String.format("i %d constraint %s state vel %f constraint vel %f",
                                i, name, stateV, maxV));
            }
        }
    }

    /** Verify acceleration constraints. */
    private void verifyAccelConstraints(TrajectorySE2 trajectory, List<TimingConstraint> constraints) {
        double margin = 1e-12;
        for (int i = 0; i < trajectory.length(); ++i) {
            TrajectorySE2Entry state = trajectory.getPoint(i);
            for (TimingConstraint constraint : constraints) {
                String name = constraint.getClass().getSimpleName();
                double stateAccel = state.point().accel();
                double maxAccel = constraint.maxAccel(state.point().point(), state.point().velocity());
                assertTrue(stateAccel - margin <= maxAccel,
                        String.format("i %d constraint %s state accel %f constraint accel %f",
                                i, name, stateAccel, maxAccel));
            }
        }
    }

    /**
     * Verify deceleration constraints.
     */
    private void verifyDecelConstraints(TrajectorySE2 trajectory, List<TimingConstraint> constraints) {
        // this huge margin still doesn't pass, so I should go fix the issue in the
        // factory.
        double margin = 0.3;
        for (int i = 0; i < trajectory.length(); ++i) {
            TrajectorySE2Entry state = trajectory.getPoint(i);
            for (TimingConstraint constraint : constraints) {
                String name = constraint.getClass().getSimpleName();
                double stateAccel = state.point().accel();
                double maxDecel = constraint.maxDecel(state.point().point(), state.point().velocity());
                assertTrue(stateAccel + margin >= maxDecel,
                        String.format("i %d constraint %s state accel %f constraint decel %f",
                                i, name, stateAccel, maxDecel));
            }
        }
    }

    /**
     * Verify the acceleration computation: acceleration at the start of each
     * segment should be the difference between velocities of the endpoints.
     */
    private void verifyAccel(TrajectorySE2 trajectory) {
        double margin = 1e-12;
        for (int i0 = 0; i0 < trajectory.length() - 1; ++i0) {
            int i1 = i0 + 1;
            TrajectorySE2Entry s0 = trajectory.getPoint(i0);
            TrajectorySE2Entry s1 = trajectory.getPoint(i1);
            double dt = s1.point().time() - s0.point().time();
            double extrapolatedVelocity = s0.point().velocity() + s0.point().accel() * dt;
            assertEquals(s1.point().velocity(), extrapolatedVelocity, margin,
                    String.format("i %d state vel %f extrapolated vel %f",
                            i1, s1.point().velocity(), extrapolatedVelocity));
        }
    }

}
