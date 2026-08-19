package org.team100.cartpole;

import org.team100.math.Dynamics;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N4;

/**
 * Ignores friction, treats pole as massless with a mass on the end.
 * 
 * @see https://openmdao.github.io/dymos/examples/cart_pole/cart_pole.html
 * @see https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html
 */
public class CartPoleSimple implements Dynamics<N4, N1> {
    /** Gravity (m/s^2) */
    private static final double g = 9.8;
    /** Pole mass (kg) */
    private static final double m_p = 0.1;
    /** Cart mass (kg) */
    private static final double m_c = 1.0;
    /** Pole length (m) */
    private final double l;

    /**
     * @param l length (m)
     */
    public CartPoleSimple(double l) {
        this.l = l;
    }

    @Override
    public Matrix<N4, N1> xdot(Matrix<N4, N1> x, Matrix<N1, N1> u) {
        double xdot = x.get(1, 0);
        double theta = x.get(2, 0);
        double thetadot = x.get(3, 0);

        double sin = Math.sin(theta);
        double cos = Math.cos(theta);
        double cos2 = cos * cos;
        double thetadot2 = thetadot * thetadot;

        double xddot = (m_p * g * sin * cos - (u.get(0, 0) + m_p * l * thetadot2 * sin))
                / (m_p * cos2 - (m_p + m_c));
        double thetaddot = (g * sin - xddot * cos) / l;

        return VecBuilder.fill(
                xdot,
                xddot,
                thetadot,
                thetaddot);
    }
}
