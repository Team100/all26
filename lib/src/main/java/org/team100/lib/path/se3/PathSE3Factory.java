package org.team100.lib.path.se3;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.geometry.se3.DirectionSE3;
import org.team100.lib.spline.se3.SplineSE3;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Twist3d;

public class PathSE3Factory {
    private static final boolean DEBUG = false;
    private static final double SPLINE_SAMPLE_TOLERANCE_M = 0.02;
    private static final double SPLINE_SAMPLE_TOLERANCE_RAD = 0.2;
    private static final double TRAJECTORY_STEP_M = 0.1;
    private final double maxNorm;
    private final double maxDx;
    private final double maxDTheta;

    public PathSE3Factory() {
        this(TRAJECTORY_STEP_M,
                SPLINE_SAMPLE_TOLERANCE_M,
                SPLINE_SAMPLE_TOLERANCE_RAD);
    }

    public PathSE3Factory(
            double maxNorm,
            double maxDx,
            double maxDTheta) {
        this.maxNorm = maxNorm;
        this.maxDx = maxDx;
        this.maxDTheta = maxDTheta;
    }

    public PathSE3 fromWaypoints(List<? extends SplineSE3> splines) {
        List<PathSE3Entry> result = new ArrayList<>();
        if (splines.isEmpty())
            return new PathSE3(result);
        result.add(splines.get(0).entry(0.0));
        for (int i = 0; i < splines.size(); i++) {
            SplineSE3 s = splines.get(i);
            if (DEBUG)
                System.out.printf("SPLINE:\n%d\n%s\n", i, s);
            addEndpointOrBisect(s, result, 0, 1);
        }
        return new PathSE3(result);
    }

    private void addEndpointOrBisect(
            SplineSE3 spline,
            List<PathSE3Entry> rv,
            double s0,
            double s1) {
        double shalf = (s0 + s1) / 2;
        Pose3d p0 = spline.pose(s0);
        Pose3d phalf = spline.pose(shalf);
        Pose3d p1 = spline.pose(s1);

        // twist from p0 to p1
        Twist3d twist_full = p1.minus(p0).log();
        // twist halfway from p0 to p1
        Twist3d twist_half = GeometryUtil.scale(twist_full, 0.5);
        // point halfway from p0 to p1
        Pose3d phalf_predicted = p0.plus(twist_half.exp());
        // difference between twist and sample
        Transform3d error = phalf_predicted.minus(phalf);

        // also prohibit large changes in direction between points
        DirectionSE3 course0 = spline.waypoint(s0).course();
        DirectionSE3 course1 = spline.waypoint(s1).course();
        Twist3d courseChange = course0.minus(course1);

        // note the extra conditions to avoid points too far apart.
        // checks both translational and l2 norms
        // also checks change in course
        if (Math.abs(error.getTranslation().getNorm()) > maxDx
                || Math.abs(error.getRotation().getAngle()) > maxDTheta
                || Metrics.translationalNorm(twist_full) > maxNorm
                || Metrics.l2Norm(twist_full) > maxNorm
                || Metrics.l2Norm(courseChange) > maxNorm) {
            // add a point in between
            addEndpointOrBisect(spline, rv, s0, shalf);
            addEndpointOrBisect(spline, rv, shalf, s1);
        } else {
            // midpoint is close enough, so add the endpoint
            rv.add(spline.entry(s1));
        }
    }

}
