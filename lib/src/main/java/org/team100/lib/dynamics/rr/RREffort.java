package org.team100.lib.dynamics.rr;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/**
 * Effort for the RR example.
 * 
 * @param proximal torque Nm
 * @param distal   torque Nm
 */
public record RREffort(double t1, double t2) {

    public static RREffort fromVector(Vector<N2> v) {
        return new RREffort(v.get(0), v.get(1));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(t1, t2);
    }

}
