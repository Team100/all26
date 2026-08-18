package org.team100.lib.path.se3;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se3.DirectionSE3;
import org.team100.lib.geometry.se3.WaypointSE3;
import org.team100.lib.spline.se3.SplineSE3;
import org.team100.lib.spline.se3.SplineSE3Factory;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PathFactorySE3Test {
    private static final double DELTA = 0.001;

    @Test
    void testLinear() {
        List<WaypointSE3> waypoints = List.of(
                new WaypointSE3(
                        new Pose3d(
                                new Translation3d(),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0), 1),
                new WaypointSE3(
                        new Pose3d(
                                new Translation3d(1, 0, 0),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0), 1));
        PathSE3Factory pathFactory = new PathSE3Factory(0.1, 0.01, 0.1);
        List<SplineSE3> splines = SplineSE3Factory.splinesFromWaypoints(waypoints);
        PathSE3 path = pathFactory.fromWaypoints(splines);
        assertEquals(17, path.length());
        PathSE3Point p = path.getEntry(0).point();
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getAngle(), DELTA);
        assertEquals(0, p.headingRate().norm(), DELTA);
        p = path.getEntry(8).point();
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getAngle(), DELTA);
        assertEquals(0, p.headingRate().norm(), DELTA);
    }

}
