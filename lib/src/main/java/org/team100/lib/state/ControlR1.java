package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * One-dimensional system state, used for control, so it includes acceleration,
 * which could be part of the control output.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 * 
 * @param x position
 * @param v velocity
 * @param a acceleration
 */
public record ControlR1(double x, double v, double a) implements Interpolatable<ControlR1> {

    /** Zero acceleration */
    public ControlR1(double x, double v) {
        this(x, v, 0);
    }

    /** Zero velocity and acceleration */
    public ControlR1(double x) {
        this(x, 0, 0);
    }

    /** Zero everything */
    public ControlR1() {
        this(0, 0, 0);
    }

    /**
     * Return the model corresponding to this control, i.e. without acceleration.
     */
    public ModelR1 model() {
        return new ModelR1(x, v);
    }

    public VelocityControlR1 velocityControl() {
        return new VelocityControlR1(v, a);
    }

    public ControlR1 minus(ControlR1 other) {
        return new ControlR1(x() - other.x(), v() - other.v(), a() - other.a());
    }

    public ControlR1 plus(ControlR1 other) {
        return new ControlR1(x() + other.x(), v() + other.v(), a() + other.a());
    }

    public ControlR1 mult(double scalar) {
        return new ControlR1(x * scalar, v * scalar, a * scalar);
    }

    public boolean near(ControlR1 other, double tolerance) {
        return MathUtil.isNear(x, other.x, tolerance) &&
                MathUtil.isNear(v, other.v, tolerance);
    }

    @Override
    public ControlR1 interpolate(ControlR1 endValue, double t) {
        return new ControlR1(
                MathUtil.interpolate(x, endValue.x, t),
                MathUtil.interpolate(v, endValue.v, t),
                MathUtil.interpolate(a, endValue.a, t));
    }

    @Override
    public String toString() {
        return String.format("ControlR1(X %5.3f V %5.3f A %5.3f)", x, v, a);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof ControlR1) {
            ControlR1 rhs = (ControlR1) other;
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
