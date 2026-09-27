package org.team100.frc2026.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Areas that might be useful for autons
 */
public class AutonPositions {
    public static final Pose2d ABOVE_BALL_FIELD = new Pose2d(7.75, 7, Rotation2d.kCCW_90deg);
    public static final Pose2d MIDDLE_BALL_FIELD = new Pose2d(7.75, 4, Rotation2d.kCCW_90deg);
    public static final Pose2d BELOW_BALL_FIELD = new Pose2d(7.75, 1, Rotation2d.kCCW_90deg);

    public static final Pose2d CLIMB_LEFT = new Pose2d(1.175, 4.25, new Rotation2d(-90 * (Math.PI / 180)));
    public static final Pose2d CLIMB_RIGHT = new Pose2d(1.175, 3.1, new Rotation2d(90 * (Math.PI / 180)));

    public static final Pose2d SHOOT_LEFT = new Pose2d(2.3, 5.1, new Rotation2d(-35 * (Math.PI / 180)));
    public static final Pose2d SHOOT_CENTER = new Pose2d(2.5, 4.0, new Rotation2d(0 * (Math.PI / 180)));
    public static final Pose2d SHOOT_RIGHT = new Pose2d(2.3, 2.7, new Rotation2d(35 * (Math.PI / 180)));

    public static final Pose2d LEFT_BUMP_MID = new Pose2d(4.66, 5.5, new Rotation2d(0 * (Math.PI / 180)));
    public static final Pose2d LEFT_BUMP_PAST = new Pose2d(8, 5.5, new Rotation2d(0 * (Math.PI / 180)));
    public static final Pose2d RIGHT_BUMP_MID = new Pose2d(4.66, 2.5, new Rotation2d(0 * (Math.PI / 180)));
    public static final Pose2d LEFT_BUMP_180DEG = new Pose2d(4, 5.5, new Rotation2d(180 * (Math.PI / 180)));

    public static final Pose2d CENTER = new Pose2d(3.5, 4.0, Rotation2d.kZero);
    // The StartingPositions.java verson of this looked like it started inside of the hub, 
    // so this starts slightly in front of that, as to be a legal starting position.
    // If the StartingPositions.java version is actually a legal starting position, this should probbably be removed.
}
