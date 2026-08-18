package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * One-dimensional system state, used for system modeling. The model only
 * contains position and velocity, there's no measurement of acceleration.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 * 
 * TODO: rename this to StateR1
 * 
 * @param x position
 * @param v velocity
 */
public record ModelR1(double x, double v) implements Interpolatable<ModelR1> {

    public ModelR1(double x) {
        this(x, 0);
    }

    public ModelR1() {
        this(0, 0);
    }

    /**
     * @return the control corresponding to this measurement, with zero
     *         acceleration.
     */
    public ControlR1 control() {
        return new ControlR1(x, v, 0);
    }

    public ModelR1 minus(ModelR1 other) {
        return new ModelR1(x() - other.x(), v() - other.v());
    }

    public ModelR1 plus(ModelR1 other) {
        return new ModelR1(x() + other.x(), v() + other.v());
    }

    public ModelR1 mult(double scalar) {
        return new ModelR1(x * scalar, v * scalar);
    }

    /** Use the velocity to evolve the position. */
    public ModelR1 evolve(double dt) {
        double dx = v * dt;
        return new ModelR1(x + dx, v);
    }

    /**
     * True if not null and position and velocity are both within (the same)
     * tolerance
     */
    public boolean near(ModelR1 other, double tolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, tolerance)
                && MathUtil.isNear(v, other.v, tolerance);
    }

    /**
     * True if not null, position is within xtolerance, velocity is within
     * vtolerance.
     */
    public boolean near(ModelR1 other, double xTolerance, double vTolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, xTolerance)
                && MathUtil.isNear(v, other.v, vTolerance);
    }

    @Override
    public ModelR1 interpolate(ModelR1 endValue, double t) {
        return new ModelR1(
                MathUtil.interpolate(x, endValue.x, t),
                MathUtil.interpolate(v, endValue.v, t));

    }

    @Override
    public String toString() {
        return String.format("ModelR1(X %11.8f V %11.8f)", x, v);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof ModelR1) {
            ModelR1 rhs = (ModelR1) other;
            return this.x == rhs.x && this.v == rhs.v;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, v);
    }
}
