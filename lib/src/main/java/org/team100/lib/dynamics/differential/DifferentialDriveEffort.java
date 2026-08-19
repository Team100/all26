package org.team100.lib.dynamics.differential;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

/**
 * Here "torque" really means "wheel force" in Newtons.
 * 
 * @param F1 left force N
 * @param F2 right force N
 */
public record DifferentialDriveEffort(
        double F1, double F2) {

    public static DifferentialDriveEffort fromVector(Vector<N2> v) {
        return new DifferentialDriveEffort(v.get(0), v.get(1));
    }
}
