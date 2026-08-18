package org.team100.lib.geometry.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class WaypointSE2Test {
    private static final double DELTA = 0.001;

    @Test
    void testCourse() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d()),
                new DirectionSE2(0, 0, 1), 1);

        assertEquals(0, w0.course().x, DELTA);
        assertEquals(0, w0.course().y, DELTA);
        assertEquals(1, w0.course().theta, DELTA);
    }

}
