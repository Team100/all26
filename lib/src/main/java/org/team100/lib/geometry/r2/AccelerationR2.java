package org.team100.lib.geometry.r2;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;

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
