package org.team100.lib.path.se2;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.path.PathUtil;
import org.team100.lib.util.Math100;
import org.team100.lib.util.StrUtil;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

/**
 * Represents a point on a path in SE(2) (plane with rotation).
 * 
 * Includes a WaypointSE2, heading rate, and curvature.
 */
public class PathSE2Point {
    static final boolean DEBUG = false;
    /**
     * Pose and course.
     */
    private final WaypointSE2 m_waypoint;
    /**
     * Curvature vector. Norm is \kappa, curvature in radians per meter, which is
     * the reciprocal of the radius.
     */
    private final Vector<N2> m_K;

    /**
     * @param waypoint Pose and course in SE(2)
     * @param K        Curvature vector
     */
    public PathSE2Point(WaypointSE2 waypoint, Vector<N2> K) {
        m_waypoint = waypoint;
        m_K = K;
    }

    public WaypointSE2 waypoint() {
        return m_waypoint;
    }

    /**
     * Signed curvature, rad/m, the reciprocal of the radius of the osculating
     * circle.
     * 
     * https://en.wikipedia.org/wiki/Curvature
     */
    public double k() {
        return PathUtil.kappaSigned(m_waypoint.course().T(), m_K);
    }

    /**
     * Curvature vector, points at the center of the osculating circle, magnitude is
     * the reciprocal of the radius.
     *
     * https://en.wikipedia.org/wiki/Curvature 
     */
    public Vector<N2> K() {
        return m_K;
    }

    /**
     * R2 (xy) planar distance only (IGNORES ROTATION) so that planar
     * velocity and curvature works correctly. Not the twist arclength.
     * Not the double-geodesic L2 thing. Just XY translation hypot.
     * 
     * Always non-negative.
     */
    public double distanceCartesian(PathSE2Point other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public boolean equals(Object other) {
        if (!(other instanceof PathSE2Point)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        PathSE2Point p2dwc = (PathSE2Point) other;
        if (!m_waypoint.equals(p2dwc.m_waypoint)) {
            if (DEBUG)
                System.out.println("wrong waypoint");
            return false;
        }
        if (!Math100.epsilonEquals(waypoint().course().headingRate(),
                p2dwc.waypoint().course().headingRate())) {
            if (DEBUG)
                System.out.println("wrong heading rate");
            return false;
        }
        if (!Math100.epsilonEquals(m_K, p2dwc.m_K)) {
            if (DEBUG)
                System.out.println("wrong K");
            return false;
        }
        return true;
    }

    public String toString() {
        return String.format(
                "pose %s course %s, K %s",
                StrUtil.pose2Str(m_waypoint.pose()),
                m_waypoint.course(),
                StrUtil.vecStr(m_K));
    }

}