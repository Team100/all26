package org.team100.lib.geometry.se2_universal_cover;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.wpilib.math.geometry.Translation2d;

public class WindingWaypointSE2Test {
    private static final double DELTA = 0.001;

    @Test
    void testCourse() {
        WindingWaypointSE2 w0 = new WindingWaypointSE2(
                new WindingPose2d(new Translation2d(), 0),
                new DirectionSE2(0, 0, 1), 1);

        assertEquals(0, w0.course().x, DELTA);
        assertEquals(0, w0.course().y, DELTA);
        assertEquals(1, w0.course().theta, DELTA);
    }

}
