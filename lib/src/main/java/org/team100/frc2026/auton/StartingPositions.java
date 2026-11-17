package org.team100.frc2026.auton;

import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;

/**
 * These measurements are guesses.
 * 
 * G303: Starting positions must overlap the starting line and not contact the
 * bump.
 */
public class StartingPositions {
    /** Barely overlaps the tape */
    private static final double X = 4.0;
    public static final WindingPose2d LEFT_TRENCH = new WindingPose2d(X, 7.4, Math.PI);
    public static final WindingPose2d LEFT_TRENCH_OUTER = new WindingPose2d(X, 7.4, 0);
    public static final WindingPose2d LEFT_BUMP = new WindingPose2d(X, 5.5, Math.PI / 180);
    public static final WindingPose2d CENTER = new WindingPose2d(X, 4.0, 0);
    public static final WindingPose2d CENTER_BACK = new WindingPose2d(X - 1, 4.0, 0);
    public static final WindingPose2d RIGHT_BUMP = new WindingPose2d(X, 2.5, Math.PI);
    public static final WindingPose2d RIGHT_TRENCH = new WindingPose2d(X, 0.6, Math.PI);
    public static final WindingPose2d RIGHT_TRENCH_OUTER = new WindingPose2d(X, 0.6, 0);
}
