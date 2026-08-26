package org.team100.lib.localization;

import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.state.StateSE2;

/**
 * Run the ground truth odometry updater at the right time.
 * 
 * Similar to FreshSwerveEstimate.
 */
public class GroundTruthCache implements DoubleFunction<StateSE2> {
    private final SwerveHistory m_history;
    private final SideEffect m_odometry;

    public GroundTruthCache(
            OdometryUpdater odometry,
            SwerveHistory history) {
        m_history = history;
        m_odometry = Cache.ofSideEffect(odometry::update);
    }

    @Override
    public StateSE2 apply(double timestampS) {
        m_odometry.run();
        return m_history.apply(timestampS);
    }

}