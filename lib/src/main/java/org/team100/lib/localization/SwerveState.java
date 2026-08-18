package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveState {
    /** Estimate for position and velocity. */
    private final ModelSE2 m_state;
    /** Estimate for position uncertainty. */
    private final IsotropicNoiseSE2 m_noise;
    /** Verbatim measurement of wheel position and angle. */
    private final SwerveModulePositions m_positions;
    /** Verbatim measurement of yaw from the gyro, uncorrected. */
    private final Rotation2d m_gyroYaw;
    /** Estimate for the gyro bias (drift rate) in rad/s. */
    private final VariableR1 m_gyroBiasRad_S;

    SwerveState(
            ModelSE2 state,
            IsotropicNoiseSE2 noise,
            SwerveModulePositions positions,
            Rotation2d gyroYaw,
            VariableR1 gyroBiasRad_S) {
        m_state = state;
        m_noise = noise;
        m_positions = positions;
        m_gyroYaw = gyroYaw;
        m_gyroBiasRad_S = gyroBiasRad_S;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof SwerveState)) {
            return false;
        }
        SwerveState rec = (SwerveState) obj;
        return Objects.equals(m_positions, rec.m_positions)
                && Objects.equals(m_state, rec.m_state);
    }

    public ModelSE2 state() {
        return m_state;
    }

    public IsotropicNoiseSE2 noise() {
        return m_noise;
    }

    public SwerveModulePositions positions() {
        return m_positions;
    }

    public Rotation2d gyroYaw() {
        return m_gyroYaw;
    }

    /** Bias in rad/s */
    public VariableR1 gyroBias() {
        return m_gyroBiasRad_S;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_positions, m_state);
    }

    @Override
    public String toString() {
        return "InterpolationRecord [m_state=" + m_state
                + ", m_wheelPositions=" + m_positions + "]";
    }

}