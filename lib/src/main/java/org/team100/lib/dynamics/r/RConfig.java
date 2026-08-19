package org.team100.lib.dynamics.r;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;

/**
 * Joint configuration for the R example.
 * 
 * @param q1 rotation of joint 1 (CCW from x)
 */
public record RConfig(double q1) {
    public Vector<N1> toVector() {
        return VecBuilder.fill(q1);
    }
}
