package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2ToVectorSeries;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;
import org.team100.lib.util.ChartUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** To visualize the different ways to parameterize a spline. */
public class ParameterizationTest {
    private static final boolean DEBUG = false;

    @Test
    void testSplineStraight() {
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 0.001),
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 0), new Rotation2d(0)),
                        new DirectionSE2(1, 0, 0), 0.001));
        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(spline));
        ChartUtil.plotOverlay(series, 500);

        XYSeries sx = SplineSE2ToVectorSeries.x("x", List.of(spline));
        XYSeries sxPrime = SplineSE2ToVectorSeries.xPrime("xprime", List.of(spline));
        XYSeries sxPrimePrime = SplineSE2ToVectorSeries.xPrimePrime("xprimeprime", List.of(spline));

        XYDataset d1 = new XYSeriesCollection(sx);
        XYDataset d2 = new XYSeriesCollection(sxPrime);
        XYDataset d3 = new XYSeriesCollection(sxPrimePrime);
        ChartUtil.plotStacked(d1, d2, d3);
    }

    @Test
    void testSplineCurved() {
        SplineSE2 spline = new SplineSE2(
                new WaypointSE2(
                        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 0), new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1));

        List<VectorSeries> series = new SplineSE2ToVectorSeries(0.1).convert(List.of(spline));
        List<VectorSeries> series2 = new SplineSE2ToVectorSeries(0.01).curvature(List.of(spline));
        List<VectorSeries> all = new ArrayList<>();
        all.addAll(series);
        all.addAll(series2);
        ChartUtil.plotOverlay(all, 500);

        XYSeries sx = SplineSE2ToVectorSeries.x("x", List.of(spline));
        XYSeries sxPrime = SplineSE2ToVectorSeries.xPrime("xprime", List.of(spline));
        XYSeries sxPrimePrime = SplineSE2ToVectorSeries.xPrimePrime("xprimeprime",
                List.of(spline));

        XYDataset d1 = new XYSeriesCollection(sx);
        XYDataset d2 = new XYSeriesCollection(sxPrime);
        XYDataset d3 = new XYSeriesCollection(sxPrimePrime);
        ChartUtil.plotStacked(d1, d2, d3);
    }

    @Test
    void testTrajectory() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(2, 0.5),
                new YawRateConstraint(1, 1));
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory();
        TrajectorySE2Planner p = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1),
                new WaypointSE2(
                        new Pose2d(new Translation2d(1, 0), new Rotation2d(0)),
                        new DirectionSE2(0, 1, 0), 1));
        TrajectorySE2 trajectory = p.generateTrajectory(waypoints, 0, 0);

        // this is wrong somehow
        if (DEBUG)
            System.out.printf("TRAJECTORY\n%s\n", trajectory);

        TrajectorySE2ToVectorSeries converter = new TrajectorySE2ToVectorSeries(0.1);
        List<VectorSeries> series = converter.convert(trajectory);
        ChartUtil.plotOverlay(series, 500);
    }

}
