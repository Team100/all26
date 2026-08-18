package org.team100.lib.dynamics.r;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

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
