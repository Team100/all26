package org.team100.lib.subsystems.swerve.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DeltaSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class FieldRelativeDeltaTest {
    @Test
    void testPolarity() {
        // the delta sign is correct
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(1, 0, new Rotation2d());
        DeltaSE2 d = DeltaSE2.delta(start, end);
        assertEquals(1, d.getTranslation().getX(), 0.01);
        assertEquals(0, d.getTranslation().getY(), 0.01);
        assertEquals(0, d.getRotation().getRadians(), 0.01);
    }

    @Test
    void testWithRotation() {
        // unlike Pose2d.minus(), the rotation is independent
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(1, 0, new Rotation2d(1));
        DeltaSE2 d = DeltaSE2.delta(start, end);
        assertEquals(1, d.getTranslation().getX(), 0.01);
        assertEquals(0, d.getTranslation().getY(), 0.01);
        assertEquals(1, d.getRotation().getRadians(), 0.01);
    }

    @Test
    void testWrapping() {
        // the delta sign is correct
        Pose2d start = new Pose2d(0, 0, new Rotation2d(3));
        Pose2d end = new Pose2d(0, 0, new Rotation2d(-3));
        DeltaSE2 d = DeltaSE2.delta(start, end);
        assertEquals(0, d.getTranslation().getX(), 0.01);
        assertEquals(0, d.getTranslation().getY(), 0.01);
        assertEquals(0.283, d.getRotation().getRadians(), 0.01);
    }
}
