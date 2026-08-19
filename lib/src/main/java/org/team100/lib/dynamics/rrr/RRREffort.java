package org.team100.lib.dynamics.rrr;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * Effort for the RR example.
 * 
 * @param q1 torque Nm
 * @param q2 torque Nm
 * @param q3 torque Nm
 */
public record RRREffort(double t1, double t2, double t3) {

    public static RRREffort fromVector(Vector<N3> v) {
        return new RRREffort(v.get(0), v.get(1), v.get(2));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(t1, t2, t3);
    }

}
