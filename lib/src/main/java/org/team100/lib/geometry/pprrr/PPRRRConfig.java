package org.team100.lib.geometry.pprrr;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N5;

/**
 * Config for arm-on-swerve.
 * 
 * There's no wrist here, to prevent redundancy.
 * 
 * @param q1 swerve x (field relative)
 * @param q2 swerve y (field relative)
 * @param q3 swerve rot (field relative)
 * @param q4 shoulder
 * @param q5 elbow
 */
public record PPRRRConfig(double q1, double q2, double q3, double q4, double q5) {

    public Vector<N5> toVector() {
        return VecBuilder.fill(q1, q2, q3, q4, q5);
    }
}
