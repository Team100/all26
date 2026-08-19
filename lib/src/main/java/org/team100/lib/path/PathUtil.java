package org.team100.lib.path;

import org.team100.lib.path.se2.PathSE2;
import org.team100.lib.path.se2.PathSE2Entry;
import org.team100.lib.path.se2.PathSE2Point;
import org.team100.lib.path.se3.PathSE3Entry;
import org.team100.lib.spline.se2.ISplineSE2;
import org.team100.lib.spline.se3.SplineSE3;
import org.team100.lib.util.Math100;
import org.team100.lib.util.StrUtil;

import org.wpilib.math.util.Num;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

/**
 * Utility functions for paths, i.e. parameterized curves.
 */
public class PathUtil {
    private static final boolean DEBUG = false;

    /**
     * Interpolates by resampling the underlying spline.
     * 
     * The parameter is assumed to vary linearly between points, so the interpolant
     * here is just a fraction of that linear variation.
     */
    public static PathSE2Entry interpolate(PathSE2Entry a, PathSE2Entry b, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.parameter().s(), b.parameter().s());
        ISplineSE2 spline = null;
        double s = 0;
        if (a.parameter().spline() == b.parameter().spline()) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = a.parameter().spline();
            s = Math100.interpolate(a.parameter().s(), b.parameter().s(), x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.parameter().s() < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = a.parameter().spline();
                s = Math100.interpolate(a.parameter().s(), 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = b.parameter().spline();
                s = Math100.interpolate(0, b.parameter().s(), x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.parameter().s(), b.parameter().s(), x, s);

        if (spline != null)
            return spline.entry(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    public static PathSE3Entry interpolate(PathSE3Entry a, PathSE3Entry b, double x) {
        if (DEBUG)
            System.out.printf("this s %f other s %f\n",
                    a.parameter().s(), b.parameter().s());
        SplineSE3 spline = null;
        double s = 0;
        if (a.parameter().spline() == b.parameter().spline()) {
            // ok to interpolate using this spline
            if (DEBUG)
                System.out.println("same spline");
            spline = a.parameter().spline();
            s = Math100.interpolate(a.parameter().s(), b.parameter().s(), x);
        } else {
            // which one to use?
            // one of the endpoints should be the spline endpoint
            // which is always the zero (not the 1)
            if (b.parameter().s() < 1e-6) {
                // other one is the start, so use this one
                if (DEBUG)
                    System.out.println("use this spline");
                spline = a.parameter().spline();
                s = Math100.interpolate(a.parameter().s(), 1, x);
            } else {
                if (DEBUG)
                    System.out.println("use the other spline");
                spline = b.parameter().spline();
                s = Math100.interpolate(0, b.parameter().s(), x);
            }
        }
        if (DEBUG)
            System.out.printf("s0 %f s1 %f x %f s %f\n",
                    a.parameter().s(), b.parameter().s(), x, s);

        if (spline != null)
            return spline.entry(s);

        new Throwable().printStackTrace();
        throw new IllegalStateException();
    }

    /**
     * Samples the entire path evenly by distance. Since the spline parameterizer
     * now contains a pathwise distance limit, you shouldn't need this anymore.
     * 
     * One difference is that the bisection method doesn't provide even sampling at
     * all: if round N is almost dense enough, then round N+1 can be almost twice as
     * dense as that. Maybe that's ok.
     */
    static PathSE2Point[] resample(PathSE2 p, double step) {
        double maxDistance = p.distance(p.length() - 1);
        if (maxDistance == 0)
            throw new IllegalArgumentException("max distance must be greater than zero");
        int num_states = (int) Math.ceil(maxDistance / step) + 1;
        if (DEBUG)
            System.out.printf("resample max distance %f step %f num states %d f %f\n",
                    maxDistance, step, num_states, maxDistance / step);
        PathSE2Point[] samples = new PathSE2Point[num_states];
        for (int i = 0; i < num_states; ++i) {
            // the dtheta and curvature come from here and are never changed.
            // the values here are just interpolated from the original values.
            double d = Math.min(i * step, maxDistance);
            PathSE2Point state = p.sample(d);
            if (state == null)
                continue;
            if (DEBUG)
                System.out.printf("RESAMPLE: i=%d d=%f state=%s\n", i, d, state);
            samples[i] = state;
        }
        return samples;
    }

    /**
     * Unit tangent vector. Really just normalizes rprime.
     * 
     * See MATH.md.
     *
     * @param rprime position derivative with respect to any parameterization
     */
    public static <N extends Num> Vector<N> T(Vector<N> rprime) {
        double rprimenorm = rprime.norm();
        return rprime.div(rprimenorm);
    }

    /**
     * In R2, the scalar curvature, $\kappa$, is the norm of the curvature vector
     * (K), signed (CCW positive) with respect to the unit tangent vector (T).
     * 
     * see MATH.md.
     * 
     * https://en.wikipedia.org/wiki/Curvature
     */
    public static double kappaSigned(Vector<N2> T, Vector<N2> K) {
        // which direction is K from T?
        double det = T.get(0) * K.get(1) - T.get(1) * K.get(0);
        return K.norm() * Math.signum(det);
    }

    /**
     * Curvature vector in any dimensionality.
     * 
     * Note for a spline in SE(n), the curvature describes the path in Rn, i.e. the
     * rotational part of the spline is not included.
     * 
     * See MATH.md.
     * 
     * @param rprime      position derivative with respect to any parameterization
     * @param rprimeprime second derivative
     */
    public static <N extends Num> Vector<N> K(Vector<N> rprime, Vector<N> rprimeprime) {
        if (DEBUG)
            System.out.printf("rprime %s rprimeprime %s\n",
                    StrUtil.vecStr(rprime), StrUtil.vecStr(rprimeprime));
        Vector<N> T = T(rprime);
        if (DEBUG)
            System.out.printf("T %s\n", StrUtil.vecStr(T));
        double rprimenorm = rprime.norm();
        // when rprimenorm is zero, curvature is meaningless, return zero.
        if (Math.abs(rprimenorm) < 1e-6) {
            return rprime.times(0);
        }
        Vector<N> p2 = rprimeprime.div(rprimenorm * rprimenorm);
        Vector<N> K = p2.minus(T.times(T.dot(p2)));
        if (DEBUG)
            System.out.printf("K %s\n", StrUtil.vecStr(K));
        return K;
    }

}
