package org.team100.lib.dynamics.r;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

/**
 * Joint acceleration for the R example.
 */
public record RAcceleration(double q1ddot) {
    public static RAcceleration fromVector(Vector<N1> v) {
        return new RAcceleration(v.get(0));
    }

    public static RAcceleration fromVector(Matrix<N1, N1> v) {
        return new RAcceleration(v.get(0, 0));
    }

    public Vector<N1> toVector() {
        return VecBuilder.fill(q1ddot);
    }
}
