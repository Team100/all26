package org.team100.lib.geometry.rrr;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record RRRVelocity(
        double q1dot,
        double q2dot,
        double q3dot) {

    public static RRRVelocity fromVector(Matrix<N3, N1> v) {
        return new RRRVelocity(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(
                q1dot,
                q2dot,
                q3dot);
    }

}
