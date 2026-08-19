package org.team100.lib.targeting;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.interpolation.Interpolator;

/**
 * Linear interpolation.
 * 
 * This is certainly unrealistic, don't use it for large differences.
 */
public class InterceptionInterpolator implements Interpolator<Interception> {

    @Override
    public Interception interpolate(Interception a, Interception b, double t) {
        return new Interception(
                MathUtil.lerp(a.range(), b.range(), t),
                MathUtil.lerp(a.tof(), b.tof(), t),
                MathUtil.lerp(a.targetElevation(), b.targetElevation(), t));
    }
}