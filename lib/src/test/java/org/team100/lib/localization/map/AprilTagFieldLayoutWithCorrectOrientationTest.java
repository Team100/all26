package org.team100.lib.localization.map;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** These tests use the 2025 map, will have to be updated in 2026. */
public class AprilTagFieldLayoutWithCorrectOrientationTest {
    private static final double DELTA = 0.001;

    @Test
    void testGetTagPoseRed() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d pose = layout.getTagPose(Alliance.Red, 1).get();
        // tag 1 coordinates for red
        assertEquals(0.851, pose.getX(), DELTA);
        assertEquals(7.396, pose.getY(), DELTA);
        assertEquals(1.486, pose.getZ(), DELTA);
    }

    @Test
    void testGetPoseBlue() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d pose = layout.getTagPose(Alliance.Blue, 1).get();
        // tag 1 coordinates for blue
        assertEquals(16.697, pose.getX(), DELTA);
        assertEquals(0.655, pose.getY(), DELTA);
        assertEquals(1.486, pose.getZ(), DELTA);
    }

    @Test
    void test0() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation(
                "2025-reefscape.json");
        Pose3d tag4pose = layout.getTagPose(Alliance.Red, 4).get();
        assertEquals(8.272, tag4pose.getX(), DELTA);
        assertEquals(1.914, tag4pose.getY(), DELTA);
        assertEquals(1.868, tag4pose.getZ(), DELTA);
    }

}
