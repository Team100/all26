package org.team100.lib.trajectory.se2.constraint;

import java.util.List;

import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.spline.se2.SplineSE2;
import org.team100.lib.spline.se2.SplineSE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2ToVectorSeries;
import org.team100.lib.util.ChartUtil;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class VelocityLimitRegionConstraintTest {

    /** Show that the velocity constraint is applied in the correct region */
    @Test
    void test0() {
        WaypointSE2 p0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 1), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 p1 = new WaypointSE2(
                new Pose2d(new Translation2d(5, 1), Rotation2d.kZero),
                new DirectionSE2(1, 0, 0), 1);
        List<WaypointSE2> waypoints = List.of(p0, p1);
        List<SplineSE2> splines = SplineSE2Factory.splinesFromWaypoints(waypoints);
        PathSE2Factory pathFactory = new PathSE2Factory(0.1, 0.02, 0.1);
        PathSE2 path = pathFactory.get(splines);
        List<TimingConstraint> constraints = List.of(
                new ConstantConstraint(2, 1),
                new VelocityLimitRegionConstraint(
                        new Rectangle2d(
                                new Translation2d(2, 0),
                                new Translation2d(3, 2)),
                        1));
        TrajectorySE2Factory generator = new TrajectorySE2Factory(constraints);
        TrajectorySE2 trajectory = generator.fromPath(path, 0, 0);
        XYSeries xdot = new TrajectorySE2ToVectorSeries(1.0).xdotVx("xdot", trajectory);
        XYDataset dataSet = new XYSeriesCollection(xdot);
        ChartUtil.plotStacked(dataSet);

    }

}
