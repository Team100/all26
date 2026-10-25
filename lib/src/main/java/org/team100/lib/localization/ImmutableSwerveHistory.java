package org.team100.lib.localization;

import java.util.Map;
import java.util.Set;

import org.team100.lib.state.StateSE2;
import org.team100.lib.util.TimeInterpolatableBuffer100;

/**
 * Used to publish the history to consumers, without exposing the mutable
 * history, which is only used by the estimator itself.
 * 
 * This is a copy rather than a view, because in the GTSAM case, all the history
 * estimates states are on the far side of the (somewhat slow) Java/C++ barrier,
 * and they're not interpolatable.
 */
public class ImmutableSwerveHistory implements StateSampler {

    private final TimeInterpolatableBuffer100<SwerveState> m_poseBuffer;

    public ImmutableSwerveHistory(
            Set<Map.Entry<Double, SwerveState>> entries,
            double bufferDuration) {
        if (entries.isEmpty())
            throw new IllegalArgumentException();
        SwerveStateInterpolator interpolator = new SwerveStateInterpolator();
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                entries, interpolator, bufferDuration);
    }

    /** Interpolates. */
    public StateSE2 get(double timestampSeconds) {
        return m_poseBuffer.get(timestampSeconds).state();
    }

    /** Return all entries. This is for visualization. */
    public Map<Double, SwerveState> all() {
        return m_poseBuffer.all();
    }

}
