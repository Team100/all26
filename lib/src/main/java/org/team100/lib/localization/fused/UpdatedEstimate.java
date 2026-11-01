package org.team100.lib.localization.fused;

import java.util.Map;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.uncertainty.NoisyStateSE2;

/**
 * Provides cached, up-to-date estimates.
 */
public class UpdatedEstimate implements StateEstimator {
    private final StateEstimator m_estimator;
    private final ObjectCache<ImmutableSwerveHistory> m_immutable;

    public UpdatedEstimate(StateEstimator estimator) {
        m_estimator = estimator;
        m_immutable = Cache.of(this::run);
    }

    private ImmutableSwerveHistory run() {
        return m_estimator.update(Takt.get());
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        return m_estimator.update(timestamp);
    }

    /** Interpolates. */
    @Override
    public NoisyStateSE2 getState(double timestampSeconds) {
        return m_immutable.get().get(timestampSeconds);
    }

    /** Return all entries. This is for visualization. */
    @Override
    public Map<Double, NoisyStateSE2> all() {
        return m_immutable.get().all();
    }

    @Override
    public void reset(double timestampS, NoisyStateSE2 state) {
        m_estimator.reset(timestampS, state);
        m_immutable.reset();
    }

    @Override
    public void close() {
        m_estimator.close();
    }
}
