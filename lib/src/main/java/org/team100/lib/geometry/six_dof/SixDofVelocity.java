package org.team100.lib.geometry.six_dof;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N6;

public record SixDofVelocity(
        double q1dot,
        double q2dot,
        double q3dot,
        double q4dot,
        double q5dot,
        double q6dot) {

    public static SixDofVelocity fromVector(Matrix<N6, N1> v) {
        return new SixDofVelocity(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0),
                v.get(3, 0),
                v.get(4, 0),
                v.get(5, 0));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(
                q1dot,
                q2dot,
                q3dot,
                q4dot,
                q5dot,
                q6dot);
    }
}
