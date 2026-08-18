package org.team100.lib.geometry.se2;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;

/**
 * A direction (i.e. unit-length vector) in the SE(2) manifold, describing the
 * evolution of Pose2d over some parameterization, including both translational
 * and rotational motion.
 * 
 * SE(2) is the space of rigid transformations in two dimensions (thus SE(2) is
 * three-dimensional, x, y, and theta).
 * 
 * This is useful for representing spline controls for Pose2d.
 * 
 * This is exactly a unit-length Twist2d.
 */
public class DirectionSE2 {
    private static final boolean DEBUG = false;

    public final double x;
    public final double y;
    public final double theta;

    public DirectionSE2(double px, double py, double ptheta) {
        double h = Math.sqrt(px * px + py * py + ptheta * ptheta);
        if (h < 1e-6)
            throw new IllegalArgumentException("zero direction is not allowed");
        x = px / h;
        y = py / h;
        theta = ptheta / h;
    }

    public Twist2d minus(DirectionSE2 other) {
        return new Twist2d(x - other.x, y - other.y, theta - other.theta);
    }

    public double normL2() {
        return Math.sqrt(x * x + y * y + theta * theta);
    }

    /** Cartesian part of direction, as an old-fashioned Rotation2d */
    public Rotation2d toRotation() {
        return new Rotation2d(x, y);
    }

    /** Unit tangent vector is the cartesian part. */
    public Vector<N2> T() {
        Rotation2d rot = toRotation();
        return VecBuilder.fill(rot.getCos(), rot.getSin());
    }

    /**
     * Rate of change of heading relative to translation, rad/m.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double headingRate() {
        double hypot = Math.hypot(x, y);
        if (hypot < 1e-6)
            return 0;
        return theta / hypot;
    }

    /** In the direction of the specified angle in radians, without rotation */
    public static DirectionSE2 irrotational(double rad) {
        return fromDirections(rad, 0);
    }

    /** In the direction of the specified angle, without rotation */
    public static DirectionSE2 irrotational(Rotation2d angle) {
        return fromDirections(angle, 0);
    }

    /** In the direction of the specified angle in radians, while rotating */
    public static DirectionSE2 fromDirections(double rad, double theta) {
        return fromDirections(new Rotation2d(rad), theta);
    }

    /** In the direction of the specified angle, while rotating */
    public static DirectionSE2 fromDirections(Rotation2d angle, double theta) {
        return new DirectionSE2(angle.getCos(), angle.getSin(), theta);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof DirectionSE2)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        DirectionSE2 other = (DirectionSE2) obj;
        if (!Math100.epsilonEquals(other.x, x)) {
            if (DEBUG)
                System.out.println("wrong x");
            return false;
        }
        if (!Math100.epsilonEquals(other.y, y)) {
            if (DEBUG)
                System.out.println("wrong y");
            return false;
        }
        if (!Math100.epsilonEquals(other.theta, theta)) {
            if (DEBUG)
                System.out.println("wrong theta");
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return String.format("%5.3f %5.3f %5.3f", x, y, theta);
    }

}
