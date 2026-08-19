package org.team100.lib.geometry.rrr;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

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
