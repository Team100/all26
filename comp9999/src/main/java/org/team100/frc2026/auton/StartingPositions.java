package org.team100.frc2026.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * These measurements are guesses.
 * 
 * G303: Starting positions must overlap the starting line and not contact the
 * bump.
 */
public class StartingPositions {
    /** Barely overlaps the tape */
    private static final double X = 4.0;
    public static final Pose2d LEFT_TRENCH = new Pose2d(X, 7.4, new Rotation2d(180 * (Math.PI / 180)));
    public static final Pose2d LEFT_BUMP = new Pose2d(X, 5.5, new Rotation2d(180 * (Math.PI / 180)));
    public static final Pose2d CENTER = new Pose2d(X, 4.0, Rotation2d.kZero);
    public static final Pose2d RIGHT_BUMP = new Pose2d(X, 2.5, new Rotation2d(180 * (Math.PI / 180)));
    public static final Pose2d RIGHT_TRENCH = new Pose2d(X, 0.6, new Rotation2d(180 * (Math.PI / 180)));
}
