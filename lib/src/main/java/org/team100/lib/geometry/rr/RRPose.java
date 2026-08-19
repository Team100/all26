package org.team100.lib.geometry.rr;

import org.wpilib.math.geometry.Pose2d;

/**
 * @param p1 pose at joint 1
 * @param p2 pose at joint 2
 * @param p3 pose at tool center point
 */
public record RRPose(Pose2d p1, Pose2d p2, Pose2d p3) {

}
