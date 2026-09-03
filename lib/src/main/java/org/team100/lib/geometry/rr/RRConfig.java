package org.team100.lib.geometry.rr;

import java.util.List;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.MathUtil;

/**
 * Joint configuration for the RR example.
 * 
 * @param q1 rotation of joint 1 ("proximal", "shoulder"), CCW rad from x
 * @param q2 rotation of joint 2 ("distal", "elbow"),CCW rad from link 1
 */
public record RRConfig(double q1, double q2) {
    // distance metric scale factors
    // shoulder movements are expensive
    static final double s1 = 3.0;
    // elbow movements are less expensive
    static final double s2 = 2.0;

    /**
     * For now, euclidean with weights.
     * 
     * You can change these weights to change how configs are selected, based on
     * their "nearness" to the current pose.
     * 
     * See https://arxiv.org/pdf/1808.03891
     */
    public double distance(RRConfig other) {
        double l2 = 0;
        l2 += s1 * Math.pow(q1 - other.q1, 2);
        l2 += s2 * Math.pow(q2 - other.q2, 2);
        return Math.sqrt(l2);
    }

    /**
     * Interpolate in configuration space, never crossing pi.
     * 
     * @param d measured using the distance metric.
     */
    public static RRConfig interpolate(RRConfig a, RRConfig b, double d) {
        double s = d / a.distance(b);
        return new RRConfig(
                MathUtil.lerp(a.q1(), b.q1(), s),
                MathUtil.lerp(a.q2(), b.q2(), s));
    }

    /**
     * Unit vector points from a to b in unscaled vector space.
     * The length of the vector is one, using the RRConfig distance metric.
     */
    public static Vector<N2> unit(RRConfig a, RRConfig b) {
        return b.minus(a).toVector().div(a.distance(b));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1, q2);
    }

    public static RRConfig fromVector(Vector<N2> v) {
        return new RRConfig(v.get(0), v.get(1));
    }

    public static RRConfig fromVector(Matrix<N2, N1> v) {
        return new RRConfig(v.get(0, 0), v.get(1, 0));
    }

    public RRConfig plus(RRConfig other) {
        return new RRConfig(q1 + other.q1, q2 + other.q2);
    }

    public RRConfig minus(RRConfig other) {
        return new RRConfig(q1 - other.q1, q2 - other.q2);
    }

    /**
     * Choose config "closest" to q0, using the (non-Euclidean) config distance
     * metric.
     */
    public static RRConfig getBest(List<RRConfig> qAll, RRConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        RRConfig best = qAll.get(0);
        for (RRConfig q : qAll) {
            double d = q0.distance(q);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
    }
}
