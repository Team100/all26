package org.team100.lib.geometry.rr;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;

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

    public double norm() {
        return Math.sqrt(q1dot * q1dot + q2dot * q2dot);
    }

    /**
     * a = (v1 - v0) / dt
     */
    public RRAcceleration accel(RRVelocity prev, double dt) {
        return RRAcceleration.fromVector(
                toVector().minus(prev.toVector()).div(dt));
    }

    /**
     * v1 = v0 + a dt
     */
    public static RRVelocity evolve(RRVelocity v0, RRAcceleration a, double dt) {
        return RRVelocity.fromVector(v0.toVector().plus(a.toVector().times(dt)));
    }
}
