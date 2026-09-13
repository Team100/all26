package org.team100.lib.geometry.prr;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * Joint configuration for the PRR example.
 * 
 * @param q1 extension of the P joint (+x)
 * @param q2 rotation of the first R joint (CCW from +x)
 * @param q3 rotation of the second R joint (CCW from the first)
 */
public record PRRConfig(double q1, double q2, double q3) {
    public static PRRConfig fromVector(Vector<N3> q) {
        return new PRRConfig(q.get(0), q.get(1), q.get(2));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(q1, q2, q3);
    }

    /**
     * Apply the velocity and accel for period dt.
     * 
     * Use the *previous* velocity here.
     * 
     * x = x0 + v0 dt + 1/2 a dt^2
     */
    public PRRConfig evolve(PRRVelocity jv, PRRAcceleration ja, double dt) {
        return PRRConfig.fromVector(
                toVector().plus(jv.toVector().times(dt).plus(ja.toVector().times(dt * dt / 2))));
    }

    /**
     * v = (x - x0) / dt
     */
    public PRRVelocity diff(PRRConfig c, double dt) {
        return new PRRVelocity(
                (q1 - c.q1) / dt,
                (q2 - c.q2) / dt,
                (q3 - c.q3) / dt);
    }

    /** True if any of the axes are NaN */
    public boolean isNaN() {
        return Double.isNaN(q1())
                || Double.isNaN(q2())
                || Double.isNaN(q3());
    }

    @Override
    public String toString() {
        return String.format("%6.3f %6.3f %6.3f", q1, q2, q3);
    }

}
