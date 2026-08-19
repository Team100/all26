package org.team100.lib.geometry.rr;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;

/**
 * Joint configuration for the RR example.
 * 
 * @param q1 rotation of joint 1 ("proximal", "shoulder"), CCW rad from x
 * @param q2 rotation of joint 2 ("distal", "elbow"),CCW rad from link 1
 */
public record RRConfig(double q1, double q2) {
    public Vector<N2> toVector() {
        return VecBuilder.fill(q1, q2);
    }
}
