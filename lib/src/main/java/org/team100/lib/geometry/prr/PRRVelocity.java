package org.team100.lib.geometry.prr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Joint velocities for PRR.
 * 
 * Use the jacobian to find these from a cartesian velocity.
 * 
 * @param q1dot velocity of the P joint
 * @param q2dot veloity of the first R joint
 * @param q3dot veloity of the second R joint
 */
public record PRRVelocity(double q1dot, double q2dot, double q3dot) {

    public static PRRVelocity fromVector(Vector<N3> v) {
        return new PRRVelocity(v.get(0), v.get(1), v.get(2));
    }

    public static PRRVelocity fromVector(Matrix<N3, N1> v) {
        return new PRRVelocity(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(q1dot, q2dot, q3dot);
    }

    public PRRAcceleration diff(PRRVelocity jv, double dt) {
        return new PRRAcceleration(
                (q1dot - jv.q1dot) / dt,
                (q2dot - jv.q2dot) / dt,
                (q3dot - jv.q3dot) / dt);
    }

    public Vector<N3> div(PRRVelocity jv) {
        return VecBuilder.fill(
                q1dot / jv.q1dot,
                q2dot / jv.q2dot,
                q3dot / jv.q3dot);
    }

    public double norm() {
        return Math.sqrt(q1dot * q1dot + q2dot * q2dot + q3dot * q3dot);
    }

    public PRRVelocity times(double s) {
        return new PRRVelocity(s * q1dot, s * q2dot, s * q3dot);
    }


}
