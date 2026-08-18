package org.team100.lib.geometry.se3;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * For constructing 3d splines, paths, and trajectories.
 * 
 * @param pose   location and orientation
 * @param course direction of travel (including rotation)
 * @param scale  influence of the course, relative to the spline parameter [0,1]
 */
public record WaypointSE3(Pose3d pose, DirectionSE3 course, double scale) {

}
