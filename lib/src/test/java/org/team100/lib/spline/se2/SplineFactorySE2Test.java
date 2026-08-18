package org.team100.lib.spline.se2;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SplineFactorySE2Test {

    /** Turning in place does not work. */
    @Test
    void testSpin() {
        List<WaypointSE2> waypoints = List.of(
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kZero),
                        new DirectionSE2(0, 0, 1), 1),
                new WaypointSE2(
                        new Pose2d(
                                new Translation2d(0, 0),
                                Rotation2d.kCCW_90deg),
                        new DirectionSE2(0, 0, 1), 1));
        assertThrows(IllegalArgumentException.class,
                () -> SplineSE2Factory.splinesFromWaypoints(waypoints));
    }

    /** Hard corners do not work. */
    @Test
    void testActualCorner() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new DirectionSE2(1, 0, 0), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d()),
                new DirectionSE2(0, 0, 1), 1);
        WaypointSE2 w2 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 0), new Rotation2d(1)),
                new DirectionSE2(0, 0, 1), 1);
        WaypointSE2 w3 = new WaypointSE2(
                new Pose2d(new Translation2d(1, 1), new Rotation2d()),
                new DirectionSE2(0, 1, 0), 1);
        List<WaypointSE2> waypoints = List.of(w0, w1, w2, w3);
        assertThrows(IllegalArgumentException.class,
                () -> SplineSE2Factory.splinesFromWaypoints(waypoints));
    }

}
