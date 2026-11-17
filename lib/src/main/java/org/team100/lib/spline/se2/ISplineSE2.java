package org.team100.lib.spline.se2;

import org.team100.lib.geometry.se2_universal_cover.WindingPose2d;
import org.team100.lib.geometry.se2_universal_cover.WindingWaypointSE2;
import org.team100.lib.path.se2.PathSE2Entry;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

/** Pose and curvature vector with arbitrary parameter */
public interface ISplineSE2 {
    WindingPose2d pose(double s);

    Vector<N2> K(double s);

    PathSE2Entry entry(double d);

    WindingWaypointSE2 waypoint(double s0);
}
