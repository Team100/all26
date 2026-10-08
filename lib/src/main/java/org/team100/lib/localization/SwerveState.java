package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;


/**
 * Estimates for oosition and velocity in SE2, with noise, and gyro bias.
 * 
 * Measurements of gyro and wheels, and, optionally,
 */
public class SwerveState {
    //
    // ESTIMATES
    //
    /** Estimate for position and velocity. */
    private final StateSE2 m_state;
    /** Uncertainty in the position estimate. */
    private final IsotropicNoiseSE2 m_noise;
    /** Estimate and uncertainty for the gyro bias (drift rate) in rad/s. */
    private final VariableR1 m_gyroBiasRad_S;
    //
    // MEASUREMENTS
    //
    /**
     * Verbatim measurement of wheel position and angle. This may be a ground-truth
     * measurement or it may be an interpolation created to match a vision
     * timestamp.
     */
    private final SwerveModulePositions m_positions;
    /** Verbatim measurement of yaw from the gyro, uncorrected. */
    private final Rotation2d m_gyroYaw;
    /** Verbatim measurement from camera, null for odometry-derived entries. */
    private final NoisyPose2d m_visionMeasurement;

    SwerveState(
            StateSE2 state,
            IsotropicNoiseSE2 noise,
            SwerveModulePositions positions,
            Rotation2d gyroYaw,
            VariableR1 gyroBiasRad_S,
            NoisyPose2d visionMeasurement) {
        // ESTIMATES
        m_state = state;
        m_noise = noise;
        m_gyroBiasRad_S = gyroBiasRad_S;
        // MEASUREMENTS
        m_positions = positions;
        m_gyroYaw = gyroYaw;
        m_visionMeasurement = visionMeasurement;
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

    public StateSE2 state() {
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

    public NoisyPose2d visionMeasurement() {
        return m_visionMeasurement;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_positions, m_state);
    }

    @Override
    public String toString() {
        return "SwerveState [m_state=" + m_state
                + ", m_noise=" + m_noise
                + ", m_gyroBiasRad_S=" + m_gyroBiasRad_S
                + ", m_wheelPositions=" + m_positions
                + ", m_gyroYaw=" + m_gyroYaw
                + ", m_visionMeasurement=" + m_visionMeasurement + "]";
    }

}