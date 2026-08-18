package org.team100.lib.path.se3;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se3.WaypointSE3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents a point on a path in SE(3) (3d space with rotation).
 * 
 * Includes a WaypointSE3, heading rate, and curvature.
 */
public class PathSE3Point {
    static final boolean DEBUG = false;
    /**
     * Pose and course.
     */
    private final WaypointSE3 m_waypoint;

    /**
     * The curvature vector is the path-length-derivative of the unit tangent
     * vector. It's an R3 vector here but it's constrained to the plane
     * perpendicular to the tangent vector, T, i.e. the course.
     * 
     * It points in the direction of the center of curvature.
     * https://en.wikipedia.org/wiki/Center_of_curvature
     * 
     * Its magnitude is "κ", 1/radius of osculating circle, rad/m
     * https://en.wikipedia.org/wiki/Osculating_circle
     */
    private final Vector<N3> m_K;

    /**
     * The heading rate is the path-length derivative of the heading vector.
     */
    private final Vector<N3> m_H;

    /**
     * @param waypoint
     * @param K        curvature vector
     * @param H        path-length angular velocity of heading
     */
    public PathSE3Point(
            WaypointSE3 waypoint,
            Vector<N3> K,
            Vector<N3> H) {
        m_waypoint = waypoint;
        Vector<N3> T = waypoint.course().translation();
        if (K.dot(T) > 1e-6)
            throw new IllegalArgumentException("K must be perpendicular to T");
        m_K = K;
        m_H = H;
    }

    public WaypointSE3 waypoint() {
        return m_waypoint;
    }

    public Vector<N3> curvature() {
        return m_K;
    }

    public Vector<N3> headingRate() {
        return m_H;
    }

    public double distanceCartesian(PathSE3Point other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

}
