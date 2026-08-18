package org.team100.lib.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Various math utilities.
 */
public class Math100 {
    private static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;

    /**
     * Returns the real solutions to the quadratic ax^2 + bx + c.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        double disc = b * b - 4 * a * c;
        if (DEBUG)
            System.out.printf("a %f b %f c %f disc %f\n", a, b, c, disc);
        if (epsilonEquals(disc, 0.0)) {
            return List.of(-b / (2 * a));
        } else if (disc > 0.0) {
            return List.of(
                    (-b + Math.sqrt(disc)) / (2 * a),
                    (-b - Math.sqrt(disc)) / (2 * a));
        } else {
            return new ArrayList<>();
        }
    }

    public static <N extends Num> boolean epsilonEquals(Vector<N> x, Vector<N> y) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            if (!epsilonEquals(x.get(i), y.get(i)))
                return false;
        }
        return true;
    }

    public static boolean epsilonEquals(double x, double y) {
        return epsilonEquals(x, y, EPSILON);
    }

    public static boolean epsilonEquals(double x, double y, double epsilon) {
        return Math.abs(x - y) < epsilon;
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** Inclusive */
    public static boolean inRange(double v, double min, double max) {
        return v >= min && v <= max;
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    /** Linear interpolation between a and b */
    public static double interpolate(double a, double b, double x) {
        if (x <= 0)
            return a;
        if (x >= 1)
            return b;
        x = limit(x, 0.0, 1.0);
        if (x < 1e-12)
            return a;
        if (x > (1 - 1e-12))
            return b;
        return a + (b - a) * x;
    }

    private Math100() {
    }

    /**
     * Produce an Euler angle equivalent to x but closer to measurement; might be
     * outside [-pi,pi].
     */
    public static double getMinDistance(double measurement, double x) {
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    public static Rotation2d getMinDistance(double measurement, Rotation2d x) {
        return new Rotation2d(MathUtil.angleModulus(x.getRadians() - measurement) + measurement);
    }

    /** Throw if NaN */
    public static double notNaN(double x) {
        if (Double.isNaN(x))
            throw new IllegalArgumentException("arg is NaN");
        return x;
    }

    public static double isFinite(double x) {
        if (!Double.isFinite(x))
            throw new IllegalArgumentException("arg is not finite");
        return x;
    }

    /** Throw if x is out of range. This is a more strict version of "clamp" :-) */
    public static double throwIfOutOfRange(double x, double minX, double maxX) {
        if (x < minX)
            throw new IllegalArgumentException(String.format("arg is %f which is below %f", x, minX));
        if (x > maxX)
            throw new IllegalArgumentException(String.format("arg is %f which is above %f", x, maxX));
        return x;
    }

    /**
     * Return acceleration implied by the change in velocity (v0 to v1)
     * over the distance, dx.
     * 
     * a = (v1^2 - v0^2) / 2dx
     * 
     * note dx can be negative, which implies negative time.
     * 
     * @param v0 initial velocity
     * @param v1 final velocity
     * @param dx distance
     */
    public static double accel(double v0, double v1, double dx) {
        if (Math.abs(dx) < 1e-6) {
            // prevent division by zero
            return 0;
        }

        /*
         * a = dv/dt
         * v = dx/dt
         * dt = dx/v
         * a = v dv/dx
         * a = v (v1-v0)/dx
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2dx
         * a = (v1^2 - v0^2)/2dx
         */
        return (v1 * v1 - v0 * v0) / (2.0 * dx);
    }

    /**
     * Return final velocity, v1, given initial velocity, v0, and acceleration over
     * distance dx.
     * 
     * v1 = sqrt(v0^2 + 2adx)
     * 
     * note a can be negative.
     * 
     * note dx can be negative, which implies backwards time
     * 
     * @param v0 initial velocity
     * @param a  acceleration
     * @param dx distance
     * @return final velocity
     */
    public static double v1(double v0, double a, double dx) {
        /*
         * a = dv/dt
         * v = dx/dt
         * dt = dx/v
         * a = v dv/dx
         * a = v (v1-v0)/dx
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2dx
         * a = (v1^2 - v0^2)/2dx
         * 2*a*dx = v1^2 - v0^2
         * v1 = sqrt(v0^2 + 2*a*dx)
         */
        return Math.sqrt(v0 * v0 + 2.0 * a * dx);
    }

    /**
     * Return initial velocity, v0, given final velocity, v1, and acceleration over
     * distance dx.
     * 
     * v0 = sqrt(v1^2 - 2adx)
     * 
     * note a can be negative.
     * 
     * note dx can be negative, which implies backwards time
     * 
     * @param v1 final velocity
     * @param a  acceleration
     * @param dx distance
     * @return final velocity
     */

    public static double v0(double v1, double a, double dx) {
        /*
         * a = dv/dt
         * v = dx/dt
         * dt = dx/v
         * a = v dv/dx
         * a = v (v1-v0)/dx
         * v = (v0+v1)/2
         * a = (v0+v1)(v1-v0)/2dx
         * a = (v1^2 - v0^2)/2dx
         * 2*a*dx = v1^2 - v0^2
         * v0 = sqrt(v1^2 - 2*a*dx)
         */
        return Math.sqrt(v1 * v1 - 2.0 * a * dx);
    }

}
