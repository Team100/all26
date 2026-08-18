package org.team100.lib.geometry.rr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Joint accelerations for the RR example
 * 
 * @param q1ddot acceleration of the medial joint
 * @param q2ddot acceleration of the distal joint
 */
public record RRAcceleration(double q1ddot, double q2ddot) {
    public static RRAcceleration fromVector(Vector<N2> v) {
        return new RRAcceleration(v.get(0), v.get(1));
    }

    public static RRAcceleration fromVector(Matrix<N2, N1> v) {
        return new RRAcceleration(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1ddot, q2ddot);
    }
}
