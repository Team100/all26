package org.team100.lib.geometry.rrr;

import org.team100.lib.util.Math100;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * 3R config
 * 
 * @param q1 shoulder rotation
 * @param q2 elbow rotation
 * @param q3 wrist rotation
 */
public record RRRConfig(double q1, double q2, double q3) {
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
        // shoulder movements are expensive
        l2 += 3.0 * Math.pow(q1 - other.q1, 2);
        // elbow movements are less expensive
        l2 += 2.0 * Math.pow(q2 - other.q2, 2);
        // wrist movements are cheap
        l2 += 1.0 * Math.pow(q3 - other.q3, 2);
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

}
