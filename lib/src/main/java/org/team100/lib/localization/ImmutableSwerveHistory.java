package org.team100.lib.localization;

import java.util.Collection;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;

import org.team100.lib.coherence.Takt;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.util.TimeInterpolatableBuffer100;

/**
 * Used to publish the history to consumers, without exposing the mutable
 * history, which is only used by the estimator itself.
 * 
 * This is a copy rather than a view, because in the GTSAM case, all the history
 * estimates states are on the far side of the (somewhat slow) Java/C++ barrier,
 * and they're not interpolatable.
 */
public class ImmutableSwerveHistory {

    private final TimeInterpolatableBuffer100<NoisyStateSE2> m_buffer;

    public static ImmutableSwerveHistory fromEstimates(
            Collection<Map.Entry<Double, LocalizerEstimate>> entries,
            double bufferDuration) {
        if (entries.isEmpty())
            throw new IllegalArgumentException();
        return new ImmutableSwerveHistory(new TimeInterpolatableBuffer100<>(
                entries.stream().map(e -> Map.entry(e.getKey(),
                        new NoisyStateSE2(e.getValue().state(), e.getValue().noise())))
                        .collect(Collectors.toSet()),
                bufferDuration));
    }

    public static ImmutableSwerveHistory fromStates(
            Collection<Map.Entry<Double, NoisyStateSE2>> entries,
            double bufferDuration) {
        if (entries.isEmpty())
            throw new IllegalArgumentException();
        return new ImmutableSwerveHistory(new TimeInterpolatableBuffer100<>(entries, bufferDuration));
    }

    public ImmutableSwerveHistory(TimeInterpolatableBuffer100<NoisyStateSE2> buffer) {
        m_buffer = buffer;
    }

    /** Interpolates. */
    public NoisyStateSE2 get(double timestampSeconds) {
        return m_buffer.get(timestampSeconds);
    }

    /** Return all entries. This is for visualization. */
    public Map<Double, NoisyStateSE2> all() {
        return m_buffer.all();
    }

}
