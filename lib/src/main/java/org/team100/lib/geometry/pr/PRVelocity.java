package org.team100.lib.geometry.pr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Joint velocities for the PR example
 * 
 * @param q1dot velocity of the P joint
 * @param q2dot veloity of the R joint
 */
public record PRVelocity(double q1dot, double q2dot) {

    public static PRVelocity fromVector(Matrix<N2, N1> v) {
        return new PRVelocity(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1dot, q2dot);
    }
}
