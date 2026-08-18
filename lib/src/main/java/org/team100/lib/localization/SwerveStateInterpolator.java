package org.team100.lib.localization;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Use a separate interpolator class since it has some state (the kinematics).
 * 
 * Interpolates the wheel positions.
 * Integrates wheel positions to find the interpolated pose.
 * Interpolates the velocity.
 */
public class SwerveStateInterpolator implements Interpolator<SwerveState> {
    private final SwerveDriveKinematics100 m_kinematics;

    public SwerveStateInterpolator(SwerveDriveKinematics100 kinematics) {
        m_kinematics = kinematics;
    }

    @Override
    public SwerveState interpolate(
            SwerveState startValue, SwerveState endValue, double t) {
        if (t <= 0) {
            return startValue;
        }
        if (t >= 1) {
            return endValue;
        }
        // Find the new wheel distances.
        SwerveModulePositions wheelLerp = new SwerveModulePositions(
                startValue.positions().frontLeft().interpolate(
                        endValue.positions().frontLeft(), t),
                startValue.positions().frontRight().interpolate(
                        endValue.positions().frontRight(), t),
                startValue.positions().rearLeft().interpolate(
                        endValue.positions().rearLeft(), t),
                startValue.positions().rearRight().interpolate(
                        endValue.positions().rearRight(), t));

        // Create a twist to represent the change based on the interpolated
        // sensor inputs.
        SwerveModuleDeltas delta = SwerveModuleDeltas.modulePositionDelta(
                startValue.positions(), wheelLerp);
        Twist2d twist = m_kinematics.forward(delta);
        Pose2d pose = startValue.state().pose().exp(twist);

        // These lerps are wrong but maybe close enough
        VelocitySE2 startVelocity = startValue.state().velocity();
        VelocitySE2 endVelocity = endValue.state().velocity();
        VelocitySE2 velocity = startVelocity.plus(
                endVelocity.minus(startVelocity).times(t));

        ModelSE2 newState = new ModelSE2(pose, velocity);
        IsotropicNoiseSE2 newNoise = startValue.noise().interpolate(
                endValue.noise(), t);

        Rotation2d gyroLerp = startValue.gyroYaw().interpolate(
                endValue.gyroYaw(), t);
        VariableR1 gyroBiasLerp = startValue.gyroBias().interpolate(
                endValue.gyroBias(), t);

        return new SwerveState(
                newState, newNoise, wheelLerp, gyroLerp, gyroBiasLerp);
    }

}
