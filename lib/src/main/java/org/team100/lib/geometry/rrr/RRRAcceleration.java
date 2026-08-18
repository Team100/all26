package org.team100.lib.geometry.rrr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record RRRAcceleration(
        double q1ddot,
        double q2ddot,
        double q3ddot) {

    public static RRRAcceleration fromVector(Matrix<N3, N1> v) {
        return new RRRAcceleration(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(
                q1ddot,
                q2ddot,
                q3ddot);
    }

}
