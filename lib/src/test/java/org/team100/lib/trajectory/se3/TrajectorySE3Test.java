package org.team100.lib.trajectory.se3;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se3.DirectionSE3;
import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.path.se3.PathSE3Factory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.trajectory.se3.constraint.ConstantConstraintSE3;
import org.team100.lib.trajectory.se3.constraint.TimingConstraintSE3;
import org.team100.lib.util.ChartUtil3d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class TrajectorySE3Test implements Timeless {
    private static final double DELTA = 0.001;
    private static final boolean DEBUG = false;

    @Test
    void testSample() {
        Pose3d start = Pose3d.kZero;
        Pose3d end = new Pose3d(1, 0, 0, Rotation3d.kZero);

        List<WaypointSE3> waypoints = List.of(
                new WaypointSE3(start, new DirectionSE3(1, 0, 0, 0, 0, 0), 1),
                new WaypointSE3(end, new DirectionSE3(1, 0, 0, 0, 0, 0), 1));

        List<TimingConstraintSE3> constraints = List.of(new ConstantConstraintSE3());

        TrajectorySE3Factory trajectoryFactory = new TrajectorySE3Factory(constraints);
        PathSE3Factory pathFactory = new PathSE3Factory();
        TrajectorySE3Planner planner = new TrajectorySE3Planner(pathFactory, trajectoryFactory);

        TrajectorySE3 trajectory = planner.restToRest(waypoints);
        ChartUtil3d.plot3dVectorSeries(
                new TrajectorySE3ToVectorSeries(0.1).fromTrajectory(trajectory));
        if (DEBUG)
            trajectory.dump();

        assertEquals(1.610, trajectory.duration(), DELTA);
        TrajectorySE3Entry sample = trajectory.sample(0);
        assertEquals(0, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(1);
        assertEquals(0.460, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
        sample = trajectory.sample(2);
        assertEquals(1, sample.point().point().waypoint().pose().getTranslation().getX(), DELTA);
    }

}
