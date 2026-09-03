package org.team100.lib.geometry.six_dof;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.team100.lib.util.Math100;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N6;

/**
 * @param q1 base/swing
 * @param q2 shoulder/boom
 * @param q3 elbow/stick
 * @param q4 wrist roll
 * @param q5 wrist pitch
 * @param q6 tool roll
 */
public record SixDofConfig(double q1, double q2, double q3, double q4, double q5, double q6) {
    private static final boolean DEBUG = false;

    public static SixDofConfig zero() {
        return new SixDofConfig(0, 0, 0, 0, 0, 0);
    }

    /**
     * For now, euclidean with weights.
     * 
     * You can change these weights to change how configs are selected, based on
     * their "nearness" to the current pose.
     * 
     * See https://arxiv.org/pdf/1808.03891
     */
    public double distance(SixDofConfig other) {
        double l2 = 0;
        // swing movements are expensive
        l2 += 10.0 * Math.pow(q1 - other.q1, 2);
        // shoulder movements are a little less expensive
        l2 += 5.0 * Math.pow(q2 - other.q2, 2);
        // elbow movements are even less but still more than wrist
        l2 += 2.0 * Math.pow(q3 - other.q3, 2);
        // don't care very much about wrist movement
        l2 += 1.0 * Math.pow(q4 - other.q4, 2);
        l2 += 1.0 * Math.pow(q5 - other.q5, 2);
        l2 += 1.0 * Math.pow(q6 - other.q6, 2);

        return Math.sqrt(l2);
    }

    /** Interpolate in configuration space, never crossing pi. */
    public static SixDofConfig interpolate(SixDofConfig a, SixDofConfig b, double s) {
        return new SixDofConfig(
                Math100.interpolate(a.q1(), b.q1(), s),
                Math100.interpolate(a.q2(), b.q2(), s),
                Math100.interpolate(a.q3(), b.q3(), s),
                Math100.interpolate(a.q4(), b.q4(), s),
                Math100.interpolate(a.q5(), b.q5(), s),
                Math100.interpolate(a.q6(), b.q6(), s));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(q1, q2, q3, q4, q5, q6);
    }

    public static SixDofConfig fromVector(Vector<N6> v) {
        return new SixDofConfig(v.get(0), v.get(1), v.get(2), v.get(3), v.get(4), v.get(5));
    }

    public static SixDofConfig fromVector(Matrix<N6, N1> v) {
        return new SixDofConfig(v.get(0, 0), v.get(1, 0), v.get(2, 0), v.get(3, 0), v.get(4, 0), v.get(5, 0));
    }

    public static SixDofConfig fromList(List<ControlR1> setpoint) {
        return new SixDofConfig(
                setpoint.get(0).x(),
                setpoint.get(1).x(),
                setpoint.get(2).x(),
                setpoint.get(3).x(),
                setpoint.get(4).x(),
                setpoint.get(5).x());
    }

    /**
     * Unit vector points from a to b in unscaled vector space.
     * The length of the vector is one, using the RRConfig distance metric.
     */
    public static Vector<N6> unit(SixDofConfig a, SixDofConfig b) {
        return b.minus(a).toVector().div(a.distance(b));
    }

    public SixDofConfig plus(SixDofConfig other) {
        return new SixDofConfig(
                q1 + other.q1,
                q2 + other.q2,
                q3 + other.q3,
                q4 + other.q4,
                q5 + other.q5,
                q6 + other.q6);
    }

    public SixDofConfig minus(SixDofConfig other) {
        return new SixDofConfig(
                q1 - other.q1,
                q2 - other.q2,
                q3 - other.q3,
                q4 - other.q4,
                q5 - other.q5,
                q6 - other.q6);
    }

    @Override
    public String toString() {
        return String.format("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f", q1, q2, q3, q4, q5, q6);
    }

    /**
     * Choose config "closest" to q0, using the (non-Euclidean) config distance
     * metric.
     */
    public static SixDofConfig getBest(List<SixDofConfig> qAll, SixDofConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        SixDofConfig best = qAll.get(0);
        for (SixDofConfig q : qAll) {
            double d = q0.distance(q);
            if (DEBUG)
                System.out.printf("q0 %s q %s distance %6.3f\n", q0, q, d);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
    }

}
