package org.team100.lib.geometry.rrr;

import org.wpilib.math.geometry.Pose2d;

/**
 * Workspace pose of each joint. p4 is the tool center point.
 */
public record RRRPose(
        Pose2d p1,
        Pose2d p2,
        Pose2d p3,
        Pose2d p4) {

}
