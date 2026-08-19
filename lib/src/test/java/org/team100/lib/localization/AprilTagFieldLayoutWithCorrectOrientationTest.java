package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.driverstation.Alliance;

/** These tests use the 2025 map, will have to be updated in 2026. */
public class AprilTagFieldLayoutWithCorrectOrientationTest {
    private static final double DELTA = 0.001;

    @Test
    void testGetTagPoseRed() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d pose = layout.getTagPose(Alliance.RED, 1).get();
        // tag 1 coordinates for red
        assertEquals(0.851, pose.getX(), DELTA);
        assertEquals(7.396, pose.getY(), DELTA);
        assertEquals(1.486, pose.getZ(), DELTA);
    }

    @Test
    void testGetPoseBlue() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d pose = layout.getTagPose(Alliance.BLUE, 1).get();
        // tag 1 coordinates for blue
        assertEquals(16.697, pose.getX(), DELTA);
        assertEquals(0.655, pose.getY(), DELTA);
        assertEquals(1.486, pose.getZ(), DELTA);
    }
}
