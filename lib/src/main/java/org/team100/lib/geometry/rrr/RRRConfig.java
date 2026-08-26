package org.team100.lib.geometry.rrr;


import org.team100.lib.util.Math100;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;


/**
 * 3R config
 * 
 * @param q1 shoulder rotation
 * @param q2 elbow rotation
 * @param q3 wrist rotation
 */
public record RRRConfig(double q1, double q2, double q3) {
    // distance metric scale factors
    // shoulder movements are expensive
    private static final double s1 = 3.0;
    // elbow movements are less expensive
    private static final double s2 = 2.0;
    // wrist movements are cheap
    private static final double s3 = 1.0;

    /**
     * For now, euclidean with weights.
     * 
     * You can change these weights to change how configs are selected, based on
     * their "nearness" to the current pose.
     * 
     * See https://arxiv.org/pdf/1808.03891
     */
    public double distance(RRRConfig other) {
        double l2 = 0;
        l2 += s1 * Math.pow(q1 - other.q1, 2);
        l2 += s2 * Math.pow(q2 - other.q2, 2);
        l2 += s3 * Math.pow(q3 - other.q3, 2);
        return Math.sqrt(l2);
    }

    /** Interpolate in configuration space, never crossing pi. */
    public static RRRConfig interpolate(RRRConfig a, RRRConfig b, double s) {
        return new RRRConfig(
                Math100.interpolate(a.q1(), b.q1(), s),
                Math100.interpolate(a.q2(), b.q2(), s),
                Math100.interpolate(a.q3(), b.q3(), s));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(q1, q2, q3);
    }


    public static RRRConfig fromVector(Vector<N3> v) {
        return new RRRConfig(v.get(0), v.get(1), v.get(2));
    }

    public static RRRConfig fromVector(Matrix<N3, N1> v) {
        return new RRRConfig(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /**
     * Unit vector points from a to b in unscaled vector space.
     * The length of the vector is one, using the RRConfig distance metric.
     */
    public static Vector<N3> unit(RRRConfig a, RRRConfig b) {
        return b.minus(a).toVector().div(a.distance(b));
    }

    public RRRConfig plus(RRRConfig other) {
        return new RRRConfig(q1 + other.q1, q2 + other.q2, q3 + other.q3);
    }

    public RRRConfig minus(RRRConfig other) {
        return new RRRConfig(q1 - other.q1, q2 - other.q2, q3 - other.q3);
    }

}
