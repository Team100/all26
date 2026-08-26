package org.team100.lib.localization;

import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.state.StateSE2;

/**
 * Updates the vision and odometry before sampling the history.
 * 
 * Proxy the history after making sure it has received any updates that may
 * mutate it. Some clients want "fresh" estimates, and should use this class;
 * other clients only need old historical estimates, and should use the history.
 */
public class FreshSwerveEstimate implements DoubleFunction<StateSE2> {
    private final DoubleFunction<StateSE2> m_history;
    /** Side effect mutates history. */
    private final SideEffect m_vision;
    /** Side effect mutates history. */
    private final SideEffect m_odometry;

    public FreshSwerveEstimate(
            Runnable visionUpdate,
            Runnable odometryUpdate,
            DoubleFunction<StateSE2> history) {
        m_history = history;
        m_vision = Cache.ofSideEffect(visionUpdate);
        m_odometry = Cache.ofSideEffect(odometryUpdate);
    }

    /**
     * Provide the best estimate for SwerveModel at the given timestamp, first
     * making sure any pending updates from vision or odometry have been applied.
     */
    @Override
    public StateSE2 apply(double timestampS) {
        // run our dependencies if they haven't already
        m_vision.run();
        m_odometry.run();
        // query the history
        return m_history.apply(timestampS);
    }

}
