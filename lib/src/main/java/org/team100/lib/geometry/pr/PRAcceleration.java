package org.team100.lib.geometry.pr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Joint accelerations for the PR example
 * 
 * @param q1ddot acceleration of the P joint
 * @param q2ddot acceleration of the R joint
 */
public record PRAcceleration(double q1ddot, double q2ddot) {
    public static PRAcceleration fromVector(Vector<N2> v) {
        return new PRAcceleration(v.get(0), v.get(1));
    }

    public static PRAcceleration fromVector(Matrix<N2, N1> v) {
        return new PRAcceleration(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1ddot, q2ddot);
    }
}
