package org.team100.lib.geometry.r2;

import java.util.function.BiPredicate;

import edu.wpi.first.math.geometry.Translation2d;

/** True if the two translations are near each other. */
public class NearR2 implements BiPredicate<Translation2d, Translation2d> {

    private final double m_threshold;

    public NearR2(double threshold) {
        m_threshold = threshold;
    }

    @Override
    public boolean test(Translation2d a, Translation2d b) {
        return a.getDistance(b) < m_threshold;
    }

}
