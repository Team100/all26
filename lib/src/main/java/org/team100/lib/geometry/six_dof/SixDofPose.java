package org.team100.lib.geometry.six_dof;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Workspace pose of each joint. p7 is the tool center point.
 */
public record SixDofPose(
        Pose3d p1,
        Pose3d p2,
        Pose3d p3,
        Pose3d p4,
        Pose3d p5,
        Pose3d p6,
        Pose3d p7) {

}
