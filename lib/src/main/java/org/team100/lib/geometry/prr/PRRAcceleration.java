package org.team100.lib.geometry.prr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Joint accelerations for the PRR example
 * 
 * @param q1ddot acceleration of the P joint
 * @param q2ddot acceleration of the first R joint
 * @param q3ddot acceleration of the second R joint
 */
public record PRRAcceleration(double q1ddot, double q2ddot, double q3ddot) {
    public Vector<N3> toVector() {
        return VecBuilder.fill(q1ddot, q2ddot, q3ddot);
    }

    public static PRRAcceleration fromVector(Matrix<N3, N1> a) {
        return new PRRAcceleration(a.get(0, 0), a.get(1, 0), a.get(2, 0));
    }

    public Vector<N3> div(PRRAcceleration ja) {
        return VecBuilder.fill(
                q1ddot / ja.q1ddot,
                q2ddot / ja.q2ddot,
                q3ddot / ja.q3ddot);
    }

    public double norm() {
        return Math.sqrt(q1ddot * q1ddot + q2ddot * q2ddot + q3ddot * q3ddot);
    }

    public PRRAcceleration times(double s) {
        return new PRRAcceleration(s * q1ddot, s * q2ddot, s * q3ddot);
    }
}
