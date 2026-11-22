package org.team100.lib.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.trajectory.se2.TrajectorySE2Point;
import org.wpilib.math.linalg.VecBuilder;

public class ControlSE2Test {
    /** Centrifugal force */
    @Test
    void test0() {
        // moving +x
        // positive curvature
        PathSE2Point point = new PathSE2Point(
                new WindingWaypointSE2(new WindingPose2d(),
                        new DirectionSE2(1, 0, 0), 1),
                VecBuilder.fill(0, 1));
        // moving at 1 m/s
        TrajectorySE2Point pp = new TrajectorySE2Point(point, 0, 1, 0);
        ControlSE2 control = pp.control();
        assertEquals(0, control.x().a(), 0.001);
        // accelerating to the left
        assertEquals(1, control.y().a(), 0.001);
    }

    /** Centrifugal force the other way */
    @Test
    void test1() {
        // moving +x
        // negative curvature
        PathSE2Point point = new PathSE2Point(
                new WindingWaypointSE2(new WindingPose2d(), new DirectionSE2(1, 0, 0), 1),
                VecBuilder.fill(0, -1));
        // moving at 1 m/s
        TrajectorySE2Point pp = new TrajectorySE2Point(point, 0, 1, 0);
        ControlSE2 control = pp.control();
        assertEquals(0, control.x().a(), 0.001);
        // accelerating to the right
        assertEquals(-1, control.y().a(), 0.001);
    }
}
