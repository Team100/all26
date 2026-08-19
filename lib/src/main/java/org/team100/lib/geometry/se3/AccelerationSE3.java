package org.team100.lib.geometry.se3;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N6;

/**
 * The second derivative of Pose2d with respect to time,
 * in the R6 tangent space to SE(3).
 */
public record AccelerationSE3(
        double x, double y, double z, double rx, double ry, double rz) {
    public static AccelerationSE3 fromVector(Matrix<N6, N1> v) {
        return new AccelerationSE3(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0),
                v.get(3, 0),
                v.get(4, 0),
                v.get(5, 0));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(x, y, z, rx, ry, rz);
    }
}
