package org.team100.lib.geometry.r2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public record AccelerationR2(double x, double y) {
    public static AccelerationR2 fromVector(Vector<N2> v) {
        return new AccelerationR2(v.get(0), v.get(1));
    }

    public static AccelerationR2 fromVector(Matrix<N2, N1> v) {
        return new AccelerationR2(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(x, y);
    }
}
