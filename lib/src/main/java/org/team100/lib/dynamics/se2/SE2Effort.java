package org.team100.lib.dynamics.se2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

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
