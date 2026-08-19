package org.team100.lib.dynamics.se2;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * Effort in SE2, also called a "wrench."
 * 
 * @param fx force in x, N
 * @param fy force in y, N
 * @param t  torque, Nm
 */
public record SE2Effort(double fx, double fy, double t) {

    public Vector<N3> vector() {
        return VecBuilder.fill(fx, fy, t);
    }
}
