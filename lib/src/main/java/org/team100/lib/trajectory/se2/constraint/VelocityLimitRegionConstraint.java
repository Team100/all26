package org.team100.lib.trajectory.se2.constraint;

import org.team100.lib.path.se2.PathSE2Point;

import org.wpilib.math.geometry.Rectangle2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A constant velocity limit within a rectangle; no limit outside.
 */
public class VelocityLimitRegionConstraint implements TimingConstraint {
    private final Rectangle2d m_region;
    private final double m_maxV;

    public VelocityLimitRegionConstraint(Rectangle2d region, double maxV) {
        if (maxV < 0)
            throw new IllegalArgumentException();
        m_region = region;
        m_maxV = maxV;
    }

    @Override
    public double maxV(PathSE2Point point) {
        final Translation2d translation = point.waypoint().pose().getTranslation();
        if (m_region.contains(translation))
            return m_maxV;
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocity) {
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return Double.NEGATIVE_INFINITY;
    }

}