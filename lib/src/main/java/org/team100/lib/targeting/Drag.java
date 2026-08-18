package org.team100.lib.targeting;

import java.util.function.UnaryOperator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

/**
 * Newton drag is proportional to the square of velocity, using the drag
 * coefficient cd and cross-sectional area A.
 * 
 * F = -0.5 * cd * rho * A * v * |v|
 * 
 * The rotation here is subject to drag, using the square of the rotational
 * velocity and an arbitrary factor, B.
 * 
 * T = -cd * rho * B * omega * |omega|
 * 
 * This also includes the Magnus effect, a force proportional to the
 * cross-product of translational and rotational velocities, using the lift
 * coefficient cl.
 * 
 * F = cl * rho* A * (v \cross omega)
 * 
 * This uses notation from here:
 * 
 * https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
 * 
 * More references:
 * 
 * https://mujoco.readthedocs.io/en/3.2.1/computation/fluid.html
 * https://www.scirp.org/journal/paperinformation?paperid=55623
 */
public class Drag implements UnaryOperator<Matrix<N6, N1>> {
    private static final boolean DEBUG = false;
    /** Air density, kg/m^3 */
    private static final double RHO = 1.225;
    /** Gravity, m/s^2 */
    private static final double G = 9.81;

    private final double mu;
    private final double nu;
    private final double xi;

    /**
     * @param cd drag coefficient, for translation and rotation drag forces
     * @param cl lift coefficient, for Magnus force
     * @param A  area, m^2
     * @param m  mass, kg
     * @param B  rotational drag fudge factor
     */
    public Drag(double cd, double cl, double A, double m, double B) {
        mu = cd * RHO * A / (2 * m);
        nu = cd * RHO * B;
        xi = cl * RHO * A / (2 * m);
    }

    /**
     * The time derivative of state.
     * 
     * @param x the current state: (x, y, theta, vx, vy, omega)
     */
    public Matrix<N6, N1> apply(Matrix<N6, N1> x) {
        double vx = x.get(3, 0);
        double vy = x.get(4, 0);
        double omega = x.get(5, 0);
        double v = Math.sqrt(vx * vx + vy * vy);
        double ax = -mu * vx * v - xi * omega * vy;
        double ay = -G - mu * vy * v + xi * omega * vx;
        double alpha = -nu * omega * Math.abs(omega);
        Vector<N6> xdot = VecBuilder.fill(vx, vy, omega, ax, ay, alpha);
        if (DEBUG)
            System.out.printf("x %12.9f y %12.9f vx %12.9f vy %12.9f\n",
                    x.get(0, 0), x.get(1, 0), xdot.get(0, 0), xdot.get(1, 0));
        return xdot;
    }
}
