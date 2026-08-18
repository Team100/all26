package org.team100.lib.geometry.six_dof;

import edu.wpi.first.math.geometry.Pose3d;

/** Workspace pose of each joint. */
public record SphericalWristPose(Pose3d p4, Pose3d p5, Pose3d p6) {

}
