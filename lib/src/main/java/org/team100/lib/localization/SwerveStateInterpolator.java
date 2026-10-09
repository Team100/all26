package org.team100.lib.localization;

import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * This used to interpolate wheel positions to find a "twist" to apply
 * to the start pose, but we don't do it that way as of 10/4/26.
 * 
 * Now we just interpolate each field separately.
 * 
 * TODO: remove this class, make SwerveState interpolatable.
 */
public class SwerveStateInterpolator implements Interpolator<SwerveState> {

    @Override
    public SwerveState interpolate(
            SwerveState startValue, SwerveState endValue, double t) {

        // Check bounds on t
        if (t <= 0) {
            return startValue;
        }
        if (t >= 1) {
            return endValue;
        }

        StateSE2 startState = startValue.state();
        StateSE2 endState = endValue.state();

        // Interpolate the state.
        StateSE2 stateLerp = startState.interpolate(endState, t);

        // Interpolate the noise.
        IsotropicNoiseSE2 noiseLerp = startValue.noise().interpolate(
                endValue.noise(), t);

        // Interpolate the wheel positions.
        SwerveModulePositions startPositions = startValue.positions();
        SwerveModulePositions endPositions = endValue.positions();
        SwerveModulePositions wheelLerp = startPositions.interpolate(endPositions, t);

        // Interpolate the gyro measurement.
        Rotation2d gyroLerp = startValue.gyroYaw().interpolate(
                endValue.gyroYaw(), t);

        // Interpolate the gyro bias estimate.
        VariableR1 gyroBiasLerp = startValue.gyroBias().interpolate(
                endValue.gyroBias(), t);

        NoisyPose2d measurementLerp = null;
        if (startValue.visionMeasurement() != null && endValue.visionMeasurement() != null) {
            measurementLerp = startValue.visionMeasurement()
                    .interpolate(endValue.visionMeasurement(), t);
        }

        return new SwerveState(
                stateLerp, noiseLerp, wheelLerp, gyroLerp, gyroBiasLerp, measurementLerp);
    }

}
