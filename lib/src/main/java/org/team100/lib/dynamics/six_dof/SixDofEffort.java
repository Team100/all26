package org.team100.lib.dynamics.six_dof;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;

/** Torques for the Six DOF arm, Nm. */
public record SixDofEffort(
        double t1,
        double t2,
        double t3,
        double t4,
        double t5,
        double t6) {

    public static SixDofEffort fromVector(Vector<N6> v) {
        return new SixDofEffort(v.get(0), v.get(1), v.get(2), v.get(3), v.get(4), v.get(5));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(t1, t2, t3, t4, t5, t6);
    }
}
