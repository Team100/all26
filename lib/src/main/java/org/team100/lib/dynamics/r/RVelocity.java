package org.team100.lib.dynamics.r;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;

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
