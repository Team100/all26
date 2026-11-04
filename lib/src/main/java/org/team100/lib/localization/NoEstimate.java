package org.team100.lib.localization;

import java.util.Map;
import java.util.TreeMap;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Pose2d;

/** Placeholder state estimator */
public class NoEstimate implements StateEstimator {

    private final TimeInterpolatableBuffer100<LocalizerEstimate> m_estimates;

    public NoEstimate() {
        LocalizerEstimate initialEstimate = new LocalizerEstimate(
                new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                VariableR1.fromVariance(0, 1));
        m_estimates = new TimeInterpolatableBuffer100<>(
                0.2, 0, initialEstimate);
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        return new ImmutableSwerveHistory(m_estimates.entrySet(), 0.2);
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        return new NoisyStateSE2(new StateSE2(), IsotropicNoiseSE2.high());
    }

    @Override
    public void reset(double timestampS, NoisyStateSE2 state) {
    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return new TreeMap<>();
    }

    @Override
    public void close() {
    }

}
