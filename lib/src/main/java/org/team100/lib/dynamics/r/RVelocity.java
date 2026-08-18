package org.team100.lib.dynamics.r;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

/**
 * Joint velocity for the RR example.
 */
public record RVelocity(double q1dot) {
    public static RVelocity fromVector(Vector<N1> v) {
        return new RVelocity(v.get(0));
    }

    public static RVelocity fromVector(Matrix<N1, N1> v) {
        return new RVelocity(v.get(0, 0));
    }

    public Vector<N1> toVector() {
        return VecBuilder.fill(q1dot);
    }
}
