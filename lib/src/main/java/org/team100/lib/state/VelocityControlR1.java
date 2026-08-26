package org.team100.lib.state;

/**
 * For velocity control, where position is not directly
 * controlled, e.g. an intake roller or shooter drum.
 * 
 * Includes targets for both velocity and acceleration.
 * 
 * @param v velocity
 * @param a acceleration
 */
public record VelocityControlR1(double v, double a) {

    public VelocityControlR1 plus(VelocityControlR1 other) {
        return new VelocityControlR1(v + other.v, a + other.a);
    }

    public VelocityControlR1 times(double scalar) {
        return new VelocityControlR1(scalar * v, scalar * a);
    }

}
