package org.team100.lib.geometry.rr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Joint velocities for the RR example
 * 
 * @param q1dot velocity of q1, rad/s
 * @param q2dot velocity of q2, rad/s
 */
public record RRVelocity(double q1dot, double q2dot) {
    public static RRVelocity fromVector(Vector<N2> v) {
        return new RRVelocity(v.get(0), v.get(1));
    }

    public static RRVelocity fromVector(Matrix<N2, N1> v) {
        return new RRVelocity(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1dot, q2dot);
    }
}
