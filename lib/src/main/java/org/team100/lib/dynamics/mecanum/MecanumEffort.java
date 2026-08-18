package org.team100.lib.dynamics.mecanum;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

/**
 * Wheel force required of the drive wheels, in Newtons.
 * 
 * As elsewhere, we represent wheel forces as linear
 * in order to encapsulate the drive details.
 * 
 * @param fl front left force N
 * @param fr front right force N
 * @param rl rear left force N
 * @param rr rear right force N
 */
public record MecanumEffort(
        double fl, double fr, double rl, double rr) {
    public static MecanumEffort fromVector(Vector<N4> v) {
        return new MecanumEffort(
                v.get(0), v.get(1), v.get(2), v.get(3));
    }
}
