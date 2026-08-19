package org.team100.math;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.util.Num;

/**
 * Describes the evolution of the system, a function of state and control input.
 * 
 * linear:
 * xdot = Ax + Bu
 * 
 * nonlinear:
 * xdot = f(x,u)
 */
public interface Dynamics<States extends Num, Inputs extends Num> {
    /**
     * @param x state (x, xdot, theta, thetadot)
     * @param u control force on cart
     */
    Matrix<States, N1> xdot(Matrix<States, N1> x, Matrix<Inputs, N1> u);
}