package org.team100.lib.uncertainty;

import java.util.Objects;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.Interpolatable;

/** StateSE2 with isotropic noise in position. */
public class NoisyStateSE2 implements Interpolatable<NoisyStateSE2> {
    /** Estimate for position and velocity. */
    private final StateSE2 m_state;
    /** Uncertainty in the position estimate. */
    private final IsotropicNoiseSE2 m_noise;

    public NoisyStateSE2(
            StateSE2 state,
            IsotropicNoiseSE2 noise) {
        m_state = state;
        m_noise = noise;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof NoisyStateSE2)) {
            return false;
        }
        NoisyStateSE2 rec = (NoisyStateSE2) obj;
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
        return "NoisyStateSE2 [m_state=" + m_state
                + ", m_noise=" + m_noise + "]";
    }

    @Override
    public NoisyStateSE2 interpolate(NoisyStateSE2 endValue, double t) {
        StateSE2 stateLerp = state().interpolate(endValue.state(), t);
        IsotropicNoiseSE2 noiseLerp = noise().interpolate(endValue.noise(), t);
        return new NoisyStateSE2(stateLerp, noiseLerp);
    }

}
