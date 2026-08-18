package org.team100.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * For use with TimeInterpolatableBuffer (or something similar).
 * 
 * Note that the buffer class short-cuts exact matches.
 * 
 * This exposes an inconsistency (a bug IMHO) in the Rotation2d class,
 * wherein the interpolated value wraps, and the uninterpolated value does not.
 * 
 * So you'll want to wrap the buffer output, e.g. by adding kZero (which wraps
 * the value!)
 */
public class Rotation2dInterpolator implements Interpolator<Rotation2d> {
    @Override
    public Rotation2d interpolate(Rotation2d startValue, Rotation2d endValue, double t) {
        return startValue.interpolate(endValue, t);
    }
}
