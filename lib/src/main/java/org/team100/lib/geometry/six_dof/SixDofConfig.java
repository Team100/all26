package org.team100.lib.geometry.six_dof;

import org.team100.lib.util.Math100;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
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

    @Override
    public String toString() {
        return String.format("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f", q1, q2, q3, q4, q5, q6);
    }

}
