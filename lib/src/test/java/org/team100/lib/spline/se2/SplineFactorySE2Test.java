package org.team100.lib.spline.se2;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.wpilib.math.geometry.Translation2d;

public class SplineFactorySE2Test {

    /** Turning in place does not work. */
    @Test
    void testSpin() {
        List<WindingWaypointSE2> waypoints = List.of(
                new WindingWaypointSE2(
                        new WindingPose2d(new Translation2d(0, 0), 0),
                        new DirectionSE2(0, 0, 1), 1),
                new WindingWaypointSE2(
                        new WindingPose2d(new Translation2d(0, 0), Math.PI / 2),
                        new DirectionSE2(0, 0, 1), 1));
        assertThrows(IllegalArgumentException.class,
                () -> SplineSE2Factory.splinesFromWaypoints(waypoints));
    }

    /** Hard corners do not work. */
    @Test
    void testActualCorner() {
        WindingWaypointSE2 w0 = new WindingWaypointSE2(
                new WindingPose2d(new Translation2d(0, 0), 0),
                new DirectionSE2(1, 0, 0), 1);
        WindingWaypointSE2 w1 = new WindingWaypointSE2(
                new WindingPose2d(new Translation2d(1, 0), 0),
                new DirectionSE2(0, 0, 1), 1);
        WindingWaypointSE2 w2 = new WindingWaypointSE2(
                new WindingPose2d(new Translation2d(1, 0), 1),
                new DirectionSE2(0, 0, 1), 1);
        WindingWaypointSE2 w3 = new WindingWaypointSE2(
                new WindingPose2d(new Translation2d(1, 1), 0),
                new DirectionSE2(0, 1, 0), 1);
        List<WindingWaypointSE2> waypoints = List.of(w0, w1, w2, w3);
        assertThrows(IllegalArgumentException.class,
                () -> SplineSE2Factory.splinesFromWaypoints(waypoints));
    }

}
