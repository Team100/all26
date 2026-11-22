package org.team100.frc2026.auton;

import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;

/**
 * Areas that might be useful for autons
 */
public class AutonPositions {
    public static final WindingPose2d ABOVE_BALL_FIELD = new WindingPose2d(7.75, 7, Math.PI / 2);
    public static final WindingPose2d MIDDLE_BALL_FIELD = new WindingPose2d(7.75, 4, Math.PI / 2);
    public static final WindingPose2d BELOW_BALL_FIELD = new WindingPose2d(7.75, 1, Math.PI / 2);

    public static final WindingPose2d CLIMB_LEFT = new WindingPose2d(1.175, 4.25, -Math.PI / 2);
    public static final WindingPose2d CLIMB_RIGHT = new WindingPose2d(1.175, 3.1, Math.PI / 2);

    public static final WindingPose2d SHOOT_LEFT = new WindingPose2d(2.3, 5.1, Math.toRadians(-35));
    public static final WindingPose2d SHOOT_CENTER = new WindingPose2d(2.5, 4.0, 0);
    public static final WindingPose2d SHOOT_RIGHT = new WindingPose2d(2.3, 2.7, Math.toRadians(35));

    public static final WindingPose2d LEFT_BUMP_MID = new WindingPose2d(4.66, 5.5, 0);
    public static final WindingPose2d LEFT_BUMP_PAST = new WindingPose2d(8, 5.5, 0);
    public static final WindingPose2d RIGHT_BUMP_MID = new WindingPose2d(4.66, 2.5, 0);
    public static final WindingPose2d LEFT_BUMP_180DEG = new WindingPose2d(4, 5.5, Math.PI);

    public static final WindingPose2d CENTER = new WindingPose2d(3.5, 4.0, 0);
    // The StartingPositions.java verson of this looked like it started inside of
    // the hub,
    // so this starts slightly in front of that, as to be a legal starting position.
    // If the StartingPositions.java version is actually a legal starting position,
    // this should probbably be removed.
}
