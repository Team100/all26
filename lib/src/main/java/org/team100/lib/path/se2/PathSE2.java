package org.team100.lib.path.se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.path.PathUtil;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * Represents a 2d holonomic path, i.e. with heading independent from course.
 * 
 * There's no timing information here. For that, see TrajectorySE2
 */
public class PathSE2 {
    // if an interpolated point is more than this far from an endpoint,
    // it indicates the endpoints are too far apart, including too far apart
    // in rotation, which is an aspect of the path scheduling that the
    // scheduler can't see
    private static double INTERPOLATION_LIMIT = 0.3;

    private final List<PathSE2Entry> m_points;
    /**
     * Cumulative translational distance, just the xy plane, not the Twist arc
     * or anything else, just xy distance.
     */
    private final double[] m_distances;

    public PathSE2(final List<PathSE2Entry> states) {
        int n = states.size();
        m_points = new ArrayList<>(n);
        m_distances = new double[n];
        if (states.isEmpty()) {
            return;
        }
        m_distances[0] = 0.0;
        m_points.add(states.get(0));
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            m_points.add(states.get(i1));
            m_distances[i1] = m_distances[i0] + getDist(i0, i1);
        }
    }

    private double getDist(int i0, int i1) {
        Pose2d p0 = getEntry(i0).point().waypoint().pose();
        Pose2d p1 = getEntry(i1).point().waypoint().pose();
        return Metrics.translationalDistance(p0, p1);
    }

    public int length() {
        return m_points.size();
    }

    public PathSE2Entry getEntry(int index) {
        if (m_points.isEmpty())
            return null;
        return m_points.get(index);
    }

    public double distance(int index) {
        if (m_points.isEmpty())
            return 0;
        return m_distances[index];
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getEntry(i).point());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    /**
     * Walks through all the points to find the bracketing points, and then
     * interpolates between them.
     * 
     * Beware, can return null if the path is empty.
     * 
     * This is not useful for operation, maybe useful for visualization; the path
     * should have enough states so you can just look at them directly.
     * 
     * @param distance in meters, always a non-negative number.
     */
    public PathSE2Point sample(double distance) {
        if (distance >= distance(length() - 1)) {
            // off the end
            return getEntry(length() - 1).point();
        }
        if (distance <= 0.0) {
            // before the start
            return getEntry(0).point();
        }
        for (int i0 = 0; i0 < length() - 1; ++i0) {
            // walk through the points to bracket the desired distance
            int i1 = i0 + 1;
            PathSE2Entry e0 = getEntry(i0);
            PathSE2Entry e1 = getEntry(i1);
            double d0 = m_distances[i0];
            double d1 = m_distances[i1];
            double d = d1 - d0;
            if (d1 >= distance) {
                // Found the bracket.
                double s = (distance - d0) / d;
                PathSE2Entry lerp = PathUtil.interpolate(e0, e1, s);
                checkLerp(e0, e1, lerp);
                return lerp.point();
            }
        }
        return null;
    }

    /**
     * Complaint about corners.
     */
    private void checkLerp(PathSE2Entry e0, PathSE2Entry e1, PathSE2Entry lerp) {
        PathSE2Point p0 = e0.point();
        PathSE2Point p1 = e1.point();
        DirectionSE2 lerpCourse = lerp.point().waypoint().course();
        Twist2d t0 = p0.waypoint().course().minus(lerpCourse);
        Twist2d t1 = p1.waypoint().course().minus(lerpCourse);
        double l0 = Metrics.l2Norm(t0);
        double l1 = Metrics.l2Norm(t1);
        if (Math.max(l0, l1) > INTERPOLATION_LIMIT)
            System.out.printf(
                    "WARNING!  Interpolated value too far away, p0=%s, p1=%s, t0=%s t1=%s.  This usually indicates a sharp corner in the path, which is not allowed.",
                    p0, p1, t0, t1);
    }

}
