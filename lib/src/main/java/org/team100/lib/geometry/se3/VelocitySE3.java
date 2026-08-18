package org.team100.lib.geometry.se3;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

/**
 * The first derivative of Pose3d with respect to time.
 * 
 * Units are meters, radians, and seconds.
 * 
 * Everything here is in the R6 tangent space to SE(3).
 * 
 * Be careful of the context: this specifies velocity relative to some
 * coordinate system, often the global (field) one, but not always.
 */
public record VelocitySE3(
        double x, double y, double z, double rx, double ry, double rz) {

    public static VelocitySE3 fromVector(Matrix<N6, N1> v) {
        return new VelocitySE3(
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
