package org.team100.lib.spline.se2;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.path.se2.PathSE2ToVectorSeries;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.util.ChartUtil;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class OffsetSE2Test {

    /** Render spline and offset spline */
    @Test
    void test0() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        SplineSE2 toolpoint = new SplineSE2(w0, w1);
        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(List.of(toolpoint));
        // this is always zero
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.1).curvature(List.of(toolpoint));

        double length = 1.0;
        OffsetSE2 offset = new OffsetSE2(toolpoint, length);
        List<VectorSeries> series3 = new SplineSE2ToVectorSeries(0.1).convert(List.of(offset));
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new SplineSE2ToVectorSeries(0.1).curvature(List.of(offset));

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 500);
    }

    /** Render full cycloid spline and offset spline */
    @Test
    void test0a() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI, 0), new Rotation2d(-Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w3 = new WaypointSE2(
                new Pose2d(new Translation2d(3 * Math.PI / 2, 0), new Rotation2d(-Math.PI)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w4 = new WaypointSE2(
                new Pose2d(new Translation2d(2 * Math.PI, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);

        List<SplineSE2> toolpoint = SplineSE2Factory.splinesFromWaypoints(List.of(w0, w1, w2, w3, w4));

        double length = 1.0;
        List<OffsetSE2> offset = toolpoint.stream().map((x) -> new OffsetSE2(x, length)).toList();

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(toolpoint);
        // this is always zero
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.1).curvature(toolpoint);

        List<VectorSeries> series3 = new SplineSE2ToVectorSeries(0.1).convert(offset);
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new SplineSE2ToVectorSeries(0.1).curvature(offset);

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 250);
    }

    /** Render path and offset path */
    @Test
    void test1() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        SplineSE2 toolpoint = new SplineSE2(w0, w1);
        double length = 1.0;
        OffsetSE2 offset = new OffsetSE2(toolpoint, length);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.001, 0.001);
        PathSE2 toolpointPath = pathFactory.get(List.of(toolpoint));
        PathSE2 offsetPath = pathFactory.get(List.of(offset));

        List<VectorSeries> series = new PathSE2ToVectorSeries(0.1).convert(toolpointPath);

        // this is always zero
        List<VectorSeries> series2 = new PathSE2ToVectorSeries(0.1).curvature(toolpointPath);

        List<VectorSeries> series3 = new PathSE2ToVectorSeries(0.1).convert(offsetPath);
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new PathSE2ToVectorSeries(0.1).curvature(offsetPath);

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 500);
    }

    /** Render full cyloid and offset path */
    @Test
    void test1a() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI, 0), new Rotation2d(-Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w3 = new WaypointSE2(
                new Pose2d(new Translation2d(3 * Math.PI / 2, 0), new Rotation2d(-Math.PI)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w4 = new WaypointSE2(
                new Pose2d(new Translation2d(2 * Math.PI, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);

        List<SplineSE2> toolpoint = SplineSE2Factory.splinesFromWaypoints(List.of(w0, w1, w2, w3, w4));

        double length = 1.0;
        List<OffsetSE2> offset = toolpoint.stream().map((x) -> new OffsetSE2(x, length)).toList();

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.001, 0.001);
        PathSE2 toolpointPath = pathFactory.get(toolpoint);
        PathSE2 offsetPath = pathFactory.get(offset);

        List<VectorSeries> series = new PathSE2ToVectorSeries(0.1).convert(toolpointPath);

        // this is always zero
        List<VectorSeries> series2 = new PathSE2ToVectorSeries(0.1).curvature(toolpointPath);

        List<VectorSeries> series3 = new PathSE2ToVectorSeries(0.1).convert(offsetPath);
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new PathSE2ToVectorSeries(0.1).curvature(offsetPath);

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 250);
    }

    /** Render trajectory and offset trajectory */
    @Test
    void test2() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        SplineSE2 toolpoint = new SplineSE2(w0, w1);
        double length = 1.0;
        OffsetSE2 offset = new OffsetSE2(toolpoint, length);

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.001, 0.001);
        PathSE2 toolpointPath = pathFactory.get(List.of(toolpoint));
        PathSE2 offsetPath = pathFactory.get(List.of(offset));

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        TrajectorySE2 toolpointTrajectory = trajectoryFactory.fromPath(toolpointPath, 0, 0);
        TrajectorySE2 offsetTrajectory = trajectoryFactory.fromPath(offsetPath, 0, 0);

        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.1).convert(toolpointTrajectory);

        // this is always zero
        List<VectorSeries> series2 = new TrajectorySE2ToVectorSeries(0.1).accel(toolpointTrajectory);

        List<VectorSeries> series3 = new TrajectorySE2ToVectorSeries(0.1).convert(offsetTrajectory);
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new TrajectorySE2ToVectorSeries(0.1).accel(offsetTrajectory);

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 500);
    }

    /** Render the full cyloid */
    @Test
    void test2a() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI, 0), new Rotation2d(-Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w3 = new WaypointSE2(
                new Pose2d(new Translation2d(3 * Math.PI / 2, 0), new Rotation2d(-Math.PI)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w4 = new WaypointSE2(
                new Pose2d(new Translation2d(2 * Math.PI, 0), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);

        List<SplineSE2> toolpoint = SplineSE2Factory.splinesFromWaypoints(List.of(w0, w1, w2, w3, w4));

        double length = 1.0;
        List<OffsetSE2> offset = toolpoint.stream().map((x) -> new OffsetSE2(x, length)).toList();

        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.001, 0.001);
        PathSE2 toolpointPath = pathFactory.get(toolpoint);
        PathSE2 offsetPath = pathFactory.get(offset);

        List<TimingConstraint> constraints = List.of(new ConstantConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(constraints);
        TrajectorySE2 toolpointTrajectory = trajectoryFactory.fromPath(toolpointPath, 0, 0);
        TrajectorySE2 offsetTrajectory = trajectoryFactory.fromPath(offsetPath, 0, 0);

        int points = 100;
        List<VectorSeries> series = new TrajectorySE2ToVectorSeries(0.2, points).convert(toolpointTrajectory);

        // this is always zero
        List<VectorSeries> series2 = new TrajectorySE2ToVectorSeries(0.5, points).accel(toolpointTrajectory);

        List<VectorSeries> series3 = new TrajectorySE2ToVectorSeries(0.2, points).convert(offsetTrajectory);
        // curvature is infinite at the start since the course is changing
        // but the offset endpoint is not moving
        List<VectorSeries> series4 = new TrajectorySE2ToVectorSeries(0.5, points).accel(offsetTrajectory);

        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        all.addAll(series3);
        all.addAll(series4);
        ChartUtil.plotOverlay(all, 250);
    }

}
