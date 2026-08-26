package org.team100.lib.state;

import java.util.Objects;

import org.wpilib.math.interpolation.Interpolatable;
import org.wpilib.math.util.MathUtil;

/**
 * One-dimensional system state, used for system modeling. The model only
 * contains position and velocity, there's no measurement of acceleration.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 * 
 * @param x position
 * @param v velocity
 */
public record StateR1(double x, double v) implements Interpolatable<StateR1> {

    public StateR1(double x) {
        this(x, 0);
    }

    public StateR1() {
        this(0, 0);
    }

    /**
     * @return the control corresponding to this measurement, with zero
     *         acceleration.
     */
    public ControlR1 control() {
        return new ControlR1(x, v, 0);
    }

    public StateR1 minus(StateR1 other) {
        return new StateR1(x() - other.x(), v() - other.v());
    }

    public StateR1 plus(StateR1 other) {
        return new StateR1(x() + other.x(), v() + other.v());
    }

    public StateR1 mult(double scalar) {
        return new StateR1(x * scalar, v * scalar);
    }

    /** Use the velocity to evolve the position. */
    public StateR1 evolve(double dt) {
        double dx = v * dt;
        return new StateR1(x + dx, v);
    }

    /**
     * True if not null and position and velocity are both within (the same)
     * tolerance
     */
    public boolean near(StateR1 other, double tolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, tolerance)
                && MathUtil.isNear(v, other.v, tolerance);
    }

    /**
     * True if not null, position is within xtolerance, velocity is within
     * vtolerance.
     */
    public boolean near(StateR1 other, double xTolerance, double vTolerance) {
        return other != null
                && MathUtil.isNear(x, other.x, xTolerance)
                && MathUtil.isNear(v, other.v, vTolerance);
    }

    @Override
    public StateR1 interpolate(StateR1 endValue, double t) {
        return new StateR1(
                MathUtil.lerp(x, endValue.x, t),
                MathUtil.lerp(v, endValue.v, t));
    }

    @Override
    public String toString() {
        return String.format("StateR1(X %11.8f V %11.8f)", x, v);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof StateR1) {
            StateR1 rhs = (StateR1) other;
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
