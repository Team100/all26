package org.team100.lib.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Linear interpolation.
 * 
 * This is certainly unrealistic, don't use it for large differences.
 */
public class InterceptionInterpolator implements Interpolator<Interception> {

    @Override
    public Interception interpolate(Interception a, Interception b, double t) {
        return new Interception(
                MathUtil.interpolate(a.range(), b.range(), t),
                MathUtil.interpolate(a.tof(), b.tof(), t),
                MathUtil.interpolate(a.targetElevation(), b.targetElevation(), t));
    }
}