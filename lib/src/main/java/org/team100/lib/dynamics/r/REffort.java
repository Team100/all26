package org.team100.lib.dynamics.r;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;

/**
 * Effort for the R example.
 * 
 * @param t torque in Nm
 */
public record REffort(double t) {

    public static REffort fromVector(Vector<N1> v) {
        return new REffort(v.get(0));
    }

    public Vector<N1> toVector() {
        return VecBuilder.fill(t);
    }
}
