package org.team100.lib.geometry.se2;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Pose and course in SE(2).
 * 
 * Course is a DirectionSE2, which is a unit vector describing how pose
 * changes with the spline parameter.
 * 
 * For constructing splines.
 * 
 * The scale factor is somewhat like "velocity" but in the spline constructor it
 * is scaled to the total length of the spline, so it affects the shape in a
 * scale-invariant way. A common scale is 1.2, but 1.0 is often useful. I've
 * used 0.9 to make pretty good circles from four splines. Elsewhere this factor
 * is sometimes called the "magic number".
 * 
 * Our waypoints have no notion of acceleration, which means the joints in the
 * resulting trajectories are (very briefly) laterally "unloaded". I don't think
 * this makes any measurable difference in real-world movement.
 * 
 * It would be possible to have different scale factors for each side of the
 * waypoint (so one side could be straighter and the other side could be
 * curvier), but I don't think it would be worth the complexity.
 * 
 * @param pose   location and orientation
 * @param course direction of travel (in SE(2), course includes rotation)
 * @param scale  influence of the course, relative to the spline parameter [0,1]
 */
public record WaypointSE2(Pose2d pose, DirectionSE2 course, double scale) {
    private static final boolean DEBUG = false;

    /** Course without rotation, with unit scale. */
    public static WaypointSE2 irrotational(Pose2d p, double course, double scale) {
        return new WaypointSE2(p, DirectionSE2.irrotational(course), scale);
    }

    /**
     * For tank drive, heading and course are the same, with unit scale.
     * This is like the WPI non-holonomic trajectory.
     */
    public static WaypointSE2 tank(Pose2d p) {
        Rotation2d r = p.getRotation();
        return new WaypointSE2(p, new DirectionSE2(r.getCos(), r.getSin(), 0), 1.0);
    }

    @Override
    public String toString() {
        return String.format("%s %s %5.3f", pose, course, scale);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof WaypointSE2)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        WaypointSE2 other = (WaypointSE2) obj;

        if (!pose.equals(other.pose)) {
            if (DEBUG)
                System.out.println("wrong pose");
            return false;
        }

        if (!course.equals(other.course)) {
            if (DEBUG)
                System.out.println("wrong course");
            return false;
        }

        if (!Math100.epsilonEquals(scale, other.scale)) {
            if (DEBUG)
                System.out.printf("wrong scale %s %s\n", scale, other.scale);
            return false;
        }

        return true;
    }

}
