package org.team100.lib.path.se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.spline.se2.ISplineSE2;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Twist2d;

public class PathSE2Factory {
    private static final boolean DEBUG = false;
    /*
     * Maximum distance of the secant lines to the continuous spline. The resulting
     * path will have little scallops if it involves rotation. In SE(2), a constant
     * "twist" segment with rotation is a curve. If the scallops are too big, make
     * this number smaller. If the trajectories are too slow to generate, make this
     * number bigger.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    /**
     * Maximum theta error.
     */
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    /**
     * Size of steps along the path. Make this number smaller for tight curves to
     * look better. Make it bigger to make trajectories (a little) faster to
     * generate.
     */
    private static final double TRAJECTORY_STEP_M = 0.1;

    private final double m_maxNorm;
    private final double m_maxDx;
    private final double m_maxDTheta;

    public PathSE2Factory() {
        this(TRAJECTORY_STEP_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
    }

    public PathSE2Factory(
            double maxNorm,
            double maxDx,
            double maxDTheta) {
        m_maxNorm = maxNorm;
        m_maxDx = maxDx;
        m_maxDTheta = maxDTheta;
    }

    /**
     * Converts a list of SplineSE2 into a PathSE2.
     * 
     * The points are chosen so that the secant line between the points is within
     * the specified tolerance (dx, dy, dtheta) of the actual spline.
     * 
     * The trajectory scheduler consumes these points, interpolating between them
     * with straight lines.
     */
    public PathSE2 get(List<? extends ISplineSE2> splines) {
        List<PathSE2Entry> result = new ArrayList<>();
        if (splines.isEmpty())
            return new PathSE2(result);
        result.add(splines.get(0).entry(0.0));
        for (int i = 0; i < splines.size(); i++) {
            ISplineSE2 spline = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, spline);
            try {
                addEndpointOrBisect(spline, result, 0, 1);
            } catch (IllegalArgumentException ex) {
                throw new IllegalArgumentException(
                        String.format("bad direction for i=%d", i),
                        ex);
            }
        }
        return new PathSE2(result);
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ///
    ///

    /**
     * Recursive bisection to find a series of secant lines close to the real curve,
     * and with the points closer than maxNorm to each other, measured in L2 norm
     * (i.e. x, y, heading), and also course.
     * 
     * Note if the path is s-shaped, then bisection can find the middle, and then
     * believe that the secant is "close" ... which is wrong. :-)
     */
    public void addEndpointOrBisect(
            ISplineSE2 spline,
            List<PathSE2Entry> rv,
            double s0,
            double s1) {
        double shalf = (s0 + s1) / 2;
        Pose2d p0 = spline.pose(s0);
        Pose2d phalf = spline.pose(shalf);
        Pose2d p1 = spline.pose(s1);

        // twist from p0 to p1
        Twist2d twist_full = p1.minus(p0).log();
        // twist halfway from p0 to p1
        Twist2d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose2d phalf_predicted = p0.plus(twist_half.exp());
        // difference between twist and sample
        Transform2d error = phalf_predicted.minus(phalf);

        // also prohibit large changes in direction between points
        DirectionSE2 course0 = spline.waypoint(s0).course();
        DirectionSE2 course1 = spline.waypoint(s1).course();
        Twist2d courseChange = course0.minus(course1);

        // note the extra conditions to avoid points too far apart.
        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getNorm()) > m_maxDx
                || Math.abs(error.getRotation().getRadians()) > m_maxDTheta
                || Metrics.translationalNorm(twist_full) > m_maxNorm
                || Metrics.l2Norm(twist_full) > m_maxNorm
                || Metrics.l2Norm(courseChange) > m_maxNorm) {
            // add a point in between
            addEndpointOrBisect(spline, rv, s0, shalf);
            addEndpointOrBisect(spline, rv, shalf, s1);
        } else {
            // midpoint is close enough, so add the endpoint
            rv.add(spline.entry(s1));
        }
    }
}
