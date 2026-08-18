package org.team100.lib.geometry.six_dof;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

public record SixDofAcceleration(
        double q1ddot,
        double q2ddot,
        double q3ddot,
        double q4ddot,
        double q5ddot,
        double q6ddot) {

    public static SixDofAcceleration fromVector(Matrix<N6, N1> v) {
        return new SixDofAcceleration(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0),
                v.get(3, 0),
                v.get(4, 0),
                v.get(5, 0));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(
                q1ddot,
                q2ddot,
                q3ddot,
                q4ddot,
                q5ddot,
                q6ddot);
    }
}
