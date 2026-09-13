package org.team100.lib.geometry.rr;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;

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

    public static RRAcceleration solve(RRConfig x0, RRConfig x1, RRVelocity v0, double dt) {
        return RRAcceleration.fromVector(
                x1.toVector().minus(x0.toVector()).minus(v0.toVector().times(dt)).times(2 / (dt * dt)));
    }

}
