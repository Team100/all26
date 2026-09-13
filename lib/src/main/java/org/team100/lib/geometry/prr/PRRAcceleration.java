package org.team100.lib.geometry.prr;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

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

    /**
     * Find the accel to get to x1 given initial x0 and v1.
     * 
     * a = 2 * (x1 - x0 - v0dt)/dt^2
     */
    public  static PRRAcceleration solve(PRRConfig x0, PRRConfig x1, PRRVelocity v0, double dt) {
        return PRRAcceleration.fromVector(
                x1.toVector().minus(x0.toVector()).minus(v0.toVector().times(dt)).times(2 / (dt * dt)));
    }
}
