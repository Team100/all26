package org.team100.lib.geometry.se2;

import java.util.Optional;

import org.team100.lib.hid.Velocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The first derivative of Pose2d with respect to time.
 * 
 * Units are meters, radians, and seconds.
 * 
 * Everything here is in the R3 tangent space to SE(2).
 * 
 * Be careful of the context: this specifies velocity relative to some
 * coordinate system, often the global (field) one, but not always.
 * 
 * See README.md for details.
 */
public record VelocitySE2(double x, double y, double theta) {

    public static final VelocitySE2 ZERO = new VelocitySE2(0, 0, 0);

    public static VelocitySE2 velocity(Pose2d start, Pose2d end, double dt) {
        DeltaSE2 d = DeltaSE2.delta(start, end);
        return new VelocitySE2(d.getX(), d.getY(), d.getRotation().getRadians()).div(dt);
    }

    /** The cartesian part only */
    public double norm() {
        return Math.hypot(x, y);
    }

    public double l2norm() {
        return Math.sqrt(x * x + y * y + theta * theta);
    }

    public VelocitySE2 normalize() {
        double norm = norm();
        if (norm < 1e-6)
            return ZERO;
        return new VelocitySE2(x, y, theta).times(1.0 / norm);
    }

    /** Field-relative course, or empty if slower than 1 micron/sec. */
    public Optional<Rotation2d> angle() {
        if (norm() < 1e-6)
            return Optional.empty();
        return Optional.of(new Rotation2d(x, y));
    }

    public VelocitySE2 plus(VelocitySE2 other) {
        return new VelocitySE2(x + other.x, y + other.y, theta + other.theta);
    }

    /** The return type here isn't really right. */
    public VelocitySE2 minus(VelocitySE2 other) {
        return new VelocitySE2(x - other.x, y - other.y, theta - other.theta);
    }

    /** Integrate the velocity from the initial pose for time dt */
    public Pose2d integrate(Pose2d initial, double dt) {
        return new Pose2d(
                initial.getX() + x * dt,
                initial.getY() + y * dt,
                initial.getRotation().plus(new Rotation2d(theta * dt)));
    }

    /**
     * Simple backwards finite difference, componentwise.
     * Centrifugal force is not relevant here, because the inputs
     * are field-relative velocities.
     */
    public AccelerationSE2 accel(VelocitySE2 previous, double dt) {
        VelocitySE2 v = minus(previous).div(dt);
        return new AccelerationSE2(v.x(), v.y(), v.theta());
    }

    public VelocitySE2 times(double scalar) {
        return new VelocitySE2(x * scalar, y * scalar, theta * scalar);
    }

    public VelocitySE2 div(double scalar) {
        return new VelocitySE2(x / scalar, y / scalar, theta / scalar);
    }

    public VelocitySE2 times(double cartesian, double angular) {
        return new VelocitySE2(x * cartesian, y * cartesian, theta * angular);
    }

    /** Dot product of translational part. */
    public double dot(VelocitySE2 other) {
        return x * other.x + y * other.y;
    }

    /** Stopping distance at the specified acceleration. */
    public Translation2d stopping(double accel) {
        double speed = norm();
        double decelTime = speed / accel;
        // the unit here is wrong
        VelocitySE2 dx = normalize().times(0.5 * speed * decelTime);
        return new Translation2d(dx.x, dx.y);
    }

    public VelocitySE2 clamp(double maxVelocity, double maxOmega) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxVelocity) {
            ratio = maxVelocity / norm;
        }
        return new VelocitySE2(ratio * x, ratio * y, MathUtil.clamp(theta, -maxOmega, maxOmega));
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }

    public static VelocitySE2 fromVector(Vector<N3> v) {
        return new VelocitySE2(v.get(0), v.get(1), v.get(2));
    }

    public static VelocitySE2 fromVector(Matrix<N3, N1> v) {
        return new VelocitySE2(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public static VelocitySE2 fromAngle(Rotation2d r) {
        return new VelocitySE2(r.getCos(), r.getSin(), 0);
    }

    public static VelocitySE2 fromDirection(DirectionSE2 d, double magnitude) {
        return new VelocitySE2(d.x, d.y, d.theta).times(magnitude);
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }

    /** Angular velocity vector, i.e. just the theta component. */
    public Vector<N3> omegaVector() {
        return VecBuilder.fill(0, 0, theta);
    }

    /** Cartesian velocity vector, i.e. just the x and y components. */
    public Vector<N3> vVector() {
        return VecBuilder.fill(x, y, 0);
    }

    /**
     * Scales driver input to field-relative velocity.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static VelocitySE2 scale(Velocity twist, double maxSpeed, double maxRot) {
        return new VelocitySE2(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }
}