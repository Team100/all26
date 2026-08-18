package org.team100.lib.geometry.se2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The second derivative of Pose2d with respect to time.
 * 
 * Units are meters, radians, and seconds.
 * 
 * This uses the R3 tangent space, not the SE(2) manifold.
 * 
 * Be careful of the context: this specifies accel relative to some
 * coordinate system, often the global (field) one, but not always.
 *
 * See README.md for details.
 */
public record AccelerationSE2(double x, double y, double theta) {
    public VelocitySE2 integrate(double dtSec) {
        return new VelocitySE2(x * dtSec, y * dtSec, theta * dtSec);
    }

    public AccelerationSE2 plus(AccelerationSE2 other) {
        return new AccelerationSE2(x + other.x, y + other.y, theta + other.theta);
    }

    public AccelerationSE2 minus(AccelerationSE2 other) {
        return new AccelerationSE2(x - other.x, y - other.y, theta - other.theta);
    }

    public AccelerationSE2 times(double scalar) {
        return new AccelerationSE2(x * scalar, y * scalar, theta * scalar);
    }

    public AccelerationSE2 div(double scalar) {
        return new AccelerationSE2(x / scalar, y / scalar, theta / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    /** Beware, the returned accel is robot-relative but not intrinsic */
    public AccelerationSE2 toRobotRelative(Rotation2d yaw) {
        Translation2d rotated = new Translation2d(x(), y()).rotateBy(yaw.unaryMinus());
        return new AccelerationSE2(rotated.getX(), rotated.getY(), theta());
    }

    /**
     * Implements the difference in extrinsic (inertial) velocities, so this is just
     * the simple difference. If the velocities were intrinsic (noninertial) then
     * centrifugal acceleration would be included. It is not.
     */
    public static AccelerationSE2 diff(
            VelocitySE2 v1,
            VelocitySE2 v2,
            double dtSec) {
        return v2.accel(v1, dtSec);
    }

    public AccelerationSE2 clamp(double maxAccel, double maxAlpha) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxAccel) {
            ratio = maxAccel / norm;
        }
        return new AccelerationSE2(ratio * x, ratio * y, MathUtil.clamp(theta, -maxAlpha, maxAlpha));
    }

    public static AccelerationSE2 fromVector(Vector<N3> v) {
        return new AccelerationSE2(v.get(0), v.get(1), v.get(2));
    }

    public static AccelerationSE2 fromVector(Matrix<N3, N1> v) {
        return new AccelerationSE2(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}
