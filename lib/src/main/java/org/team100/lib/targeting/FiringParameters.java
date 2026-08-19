package org.team100.lib.targeting;

import org.wpilib.math.util.MathUtil;

/**
 * Shooting solution including time of flight.
 * 
 * @param range     (m) to target center, in 2d, measured on the floor
 * @param speed     (m/s) shooter drum speed
 * @param elevation (rad) of the hood, not the ball path
 * @param tof       (sec) measured with a stopwatch
 */
public record FiringParameters(
        double range, double speed, double elevation, double tof) {

    public static FiringParameters interpolate(
            FiringParameters a, FiringParameters b, double t) {
        return new FiringParameters(
                MathUtil.lerp(a.range(), b.range(), t),
                MathUtil.lerp(a.speed(), b.speed(), t),
                MathUtil.lerp(a.elevation(), b.elevation(), t),
                MathUtil.lerp(a.tof(), b.tof(), t));
    }

}
