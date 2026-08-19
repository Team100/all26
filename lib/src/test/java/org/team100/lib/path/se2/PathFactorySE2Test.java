package org.team100.lib.path.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.ChartUtil;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

public class PathFactorySE2Test implements Timeless {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.01;

    /** Preserves the tangent at the corner and so makes a little "S" */
    @Test
    void testCorner() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1, w2);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.01, 0.1);
        PathSE2 path = pathFactory.get(splines);

        assertEquals(54, path.length());
        PathSE2Point p = path.getEntry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
        p = path.getEntry(8).point();
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testLinear() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.01, 0.1);
        PathSE2 path = pathFactory.get(splines);
        assertEquals(17, path.length());
        PathSE2Point p = path.getEntry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
        p = path.getEntry(8).point();
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getRadians(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
    }

    @Test
    void testComposite() {
        // note none of the directions include rotation, so dtheta is zero at the knots.
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(1)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(2, 0), new Rotation2d(1)),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1, w2);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.01, 0.1);
        PathSE2 path = pathFactory.get(splines);
        List<VectorSeries> series = new PathSE2ToVectorSeries(0.1).convert(path);
        ChartUtil.plotOverlay(series, 500);
        assertEquals(59, path.length(), 0.001);
    }

    @Test
    void testCurve() {
        // note none of the directions include rotation, so dtheta is zero at the knots.
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), new Rotation2d(2)),
                new DirectionSE2(0, 1, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.01, 0.1);
        PathSE2 path = pathFactory.get(splines);
        List<VectorSeries> series = new PathSE2ToVectorSeries(0.1).convert(path);
        ChartUtil.plotOverlay(series, 500);
        assertEquals(57, path.length(), 0.001);
    }

    @Test
    void test() {
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1.2);
        WaypointSE2 p2 = new WaypointSE2(
                new Pose2d(new Translation2d(15, 10), Rotation2d.kZero),
                new DirectionSE2(1, 5, 0), 1.2);
        SplineSE2 s = new SplineSE2(p1, p2);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.05, 0.1);

        List<PathSE2Entry> samples = new ArrayList<>();
        samples.add(s.entry(0.0));
        pathFactory.addEndpointOrBisect(s, samples, 0, 1);

        double arclength = 0;
        PathSE2Point cur_pose = samples.get(0).point();
        for (PathSE2Entry sample : samples) {
            Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(
                                    cur_pose.waypoint().pose()),
                            sample.point().waypoint().pose()));
            arclength += Math.hypot(twist.dx, twist.dy);
            cur_pose = sample.point();
        }

        WaypointSE2 pose = cur_pose.waypoint();
        assertEquals(15.0, pose.pose().getTranslation().getX(), 0.001);
        assertEquals(10.0, pose.pose().getTranslation().getY(), 0.001);
        assertEquals(78.690, pose.course().toRotation().getDegrees(), 0.001);
        assertEquals(20.428, arclength, 0.001);
    }

    @Test
    void testDx() {
        SplineSE2 s0 = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, -1),
                                Rotation2d.kZero),
                        new DirectionSE2(1, 0, 0), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(1, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 1, 0), 1));
        List<SplineSE2> splines = List.of(s0);
        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.001, 0.001);
        PathSE2 motion = pathFactory.get(splines);
        for (int i = 0; i < motion.length(); ++i) {
            PathSE2Point p = motion.getEntry(i).point();
            if (DEBUG)
                System.out.printf("%5.3f %5.3f\n", p.waypoint().pose().getTranslation().getX(),
                        p.waypoint().pose().getTranslation().getY());
        }
    }

    /**
     * Show x as a function of the pose list index.
     * The pose list has no parameter, it's just a list
     */
    @Test
    void testPoses() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, 0), 1);
        SplineSE2 spline = new SplineSE2(w0, w1);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.02, 0.1);

        List<PathSE2Entry> poses = new ArrayList<>();
        poses.add(spline.entry(0.0));
        pathFactory.addEndpointOrBisect(spline, poses, 0, 1);

        XYSeries sx = PathSE2ToVectorSeries.x("spline", poses.stream().map(x -> x.point()).toList());
        XYDataset dataSet = new XYSeriesCollection(sx);
        ChartUtil.plotStacked(dataSet);
    }

    @Test
    void testEmpty() {
        List<SplineSE2> splines = new ArrayList<>();
        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.1, 0.1);
        PathSE2 path = pathFactory.get(splines);
        assertEquals(0, path.length(), 0.001);
    }

    /**
     * 0.15 ms on my machine.
     * 
     * See TrajectoryPlannerTest::testPerformance().
     */
    @Test
    void testPerformance() {
        // quarter-circle.
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1.2);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1.2);
        List<WaypointSE2> waypoints = List.of(w0, w1);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.05, 0.2);
        PathSE2 path = new PathSE2(new ArrayList<>());
        final int iterations = 100;
        long startTimeNs = System.nanoTime();
        for (int i = 0; i < iterations; ++i) {
            path = pathFactory.get(splines);
        }
        long endTimeNs = System.nanoTime();
        double totalDurationMs = (endTimeNs - startTimeNs) / 1000000.0;
        if (DEBUG) {
            System.out.printf("total duration ms: %5.3f\n", totalDurationMs);
            System.out.printf("duration per iteration ms: %5.3f\n", totalDurationMs / iterations);
        }
        assertEquals(33, path.length());
        PathSE2Point p = path.getEntry(4).point();
        assertEquals(0.211, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().course().headingRate(), DELTA);
    }

}
