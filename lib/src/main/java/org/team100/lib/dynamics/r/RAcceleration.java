package org.team100.lib.dynamics.r;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;

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
