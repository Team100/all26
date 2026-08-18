package org.team100.lib.dynamics.r;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

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
