package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * The quantities estimated by the localizer.
 */
public class LocalizerEstimate implements Interpolatable<LocalizerEstimate> {
    /** Estimate for position and velocity. */
    private final StateSE2 m_state;
    /** Uncertainty in the position estimate. */
    private final IsotropicNoiseSE2 m_noise;
    /** Estimate and uncertainty for the gyro bias (drift rate) in rad/s. */
    private final VariableR1 m_gyroBiasRad_S;

    public LocalizerEstimate(
            StateSE2 state,
            IsotropicNoiseSE2 noise,
            VariableR1 gyroBiasRad_S) {
        m_state = state;
        m_noise = noise;
        m_gyroBiasRad_S = gyroBiasRad_S;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof LocalizerEstimate)) {
            return false;
        }
        LocalizerEstimate rec = (LocalizerEstimate) obj;
        return Objects.equals(m_state, rec.m_state);
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_state);
    }

    public StateSE2 state() {
        return m_state;
    }

    public IsotropicNoiseSE2 noise() {
        return m_noise;
    }

    /** Bias in rad/s */
    public VariableR1 gyroBias() {
        return m_gyroBiasRad_S;
    }

    public Pose2d pose() {
        return state().pose();
    }

    public NoisyPose2d noisyPose() {
        return new NoisyPose2d(state().pose(), noise());
    }

    public VelocitySE2 velocity() {
        return state().velocity();
    }

    @Override
    public String toString() {
        return "LocalizerEstimate [m_state=" + m_state
                + ", m_noise=" + m_noise
                + ", m_gyroBiasRad_S=" + m_gyroBiasRad_S + "]";
    }

    @Override
    public LocalizerEstimate interpolate(LocalizerEstimate endValue, double t) {
        StateSE2 stateLerp = state().interpolate(endValue.state(), t);
        IsotropicNoiseSE2 noiseLerp = noise().interpolate(endValue.noise(), t);
        VariableR1 gyroBiasLerp = gyroBias().interpolate(endValue.gyroBias(), t);
        return new LocalizerEstimate(stateLerp, noiseLerp, gyroBiasLerp);
    }
}
