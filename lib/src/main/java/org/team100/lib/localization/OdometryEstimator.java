package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.function.DoubleFunction;
import java.util.function.UnaryOperator;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.fusion.CovarianceInflation;
import org.team100.lib.fusion.Fusor;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.IsotropicNoiseSE2Logger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.OdometryNoise;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Makes a SwerveState estimate with new odometry by selecting the most-recent
 * pose and applying the pose delta represented by the odometry, with the gyro
 * mixed in.
 */
public class OdometryEstimator {
    private static final boolean DEBUG = false;

    private final SwerveKinodynamics m_kinodynamics;
    private final double m_gyroWhiteNoise;
    /**
     * History entry before timestamp, the initial value for integrating odometry.
     */
    private final DoubleFunction<Entry<Double, SwerveState>> m_lowerEntry;
    /**
     * Noise source for simulation.
     * For a real robot, use UnaryOperator.identity().
     */
    private final UnaryOperator<Twist2d> m_noise;
    /**
     * Use the module positions all the time: for ground truth.
     */
    private final boolean m_alwaysUpdate;
    /**
     * Minimum variance for this fusor represents the true bias noise, aka "bias
     * instability," which is quite low.
     */
    private final Fusor m_gyroBiasFusor;
    private final Fusor m_rotationFusor;

    private final IsotropicNoiseSE2Logger m_log_prevNoise;
    private final IsotropicNoiseSE2Logger m_log_updateNoise;
    private final IsotropicNoiseSE2Logger m_log_newNoise;

    /**
     * The double function must return the entry *before* the supplied timestamp,
     * since we integrate odometry after that.
     */
    public OdometryEstimator(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            double whiteNoise, // gyro white noise
            double biasNoise, // gyro bias noise
            DoubleFunction<Entry<Double, SwerveState>> lowerEntry, // history entry before timestamp
            boolean noisy, // if true, add noise; used for simulation
            boolean alwaysUpdate) {
        LoggerFactory log = parent.type(this);
        m_kinodynamics = kinodynamics;
        m_gyroWhiteNoise = whiteNoise;
        m_lowerEntry = lowerEntry;
        m_noise = noisy ? new AddOdometryNoise() : UnaryOperator.identity();
        m_alwaysUpdate = alwaysUpdate;
        m_gyroBiasFusor = new CovarianceInflation(0.02, biasNoise);
        m_rotationFusor = new CovarianceInflation(0.02, 0.003);
        m_log_prevNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "previous noise");
        m_log_updateNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "update noise");
        m_log_newNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "new noise");
    }

    ////////////////////////////////////////////////////

    /**
     * Estimate SwerveState at the specified time, based on the measured
     * yaw and positions.
     * 
     * There is no history replay here, though it won't fail if you give it
     * out-of-order input.
     * 
     * Samples the history before the specified time, and records a new pose
     * based on the difference in wheel positions between the sample and the
     * specified positions.
     * 
     * The gyro angle overrides the odometry-derived gyro measurement, and
     * the gyro rate overrides the rate derived from the difference to the previous
     * state.
     * 
     * Can return null if it's not possible to make an estimate.
     * 
     * @param timestamp Takt time, seconds
     * @param gyroYaw   Verbatim gyro measurement
     * @param positions Verbatim drive measurement
     */
    SwerveState estimate(
            double timestamp,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {

        // The entry right before this one, the basis for integration.
        // Never interpolated.
        Entry<Double, SwerveState> lowerEntry = m_lowerEntry.apply(timestamp);

        if (lowerEntry == null) {
            // System.out.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return null;
        }

        double dt = timestamp - lowerEntry.getKey();

        SwerveState previousState = lowerEntry.getValue();
        if (dt < 0.0001) {
            // I'm not sure why this happens. In any case, the logic is deterministic so
            // there's no reason to repeat it.
            return null;
        }

        if (DEBUG)
            System.out.printf("=== compute for current time %6.3f sample time %6.3f dt %6.3f\n",
                    timestamp, lowerEntry.getKey(), dt);

        return newState(previousState, dt, gyroYaw, positions);
    }

    /**
     * Compute the new state, based on the previous state.
     */
    SwerveState newState(
            SwerveState previousState,
            double dt,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {
        if (DEBUG) {
            System.out.printf("previous x %.6f y %.6f\n",
                    previousState.state().pose().getX(), previousState.state().pose().getY());
        }
        // The SE2 "twist" increment between the previous and current poses.
        Twist2d twist = twistFromOdometry(positions, previousState.positions());

        // The verbatim gyro increment, with an uncertainty estimate.
        VariableR1 gyroIncrementRad = getGyroMeasurementRad(dt, gyroYaw, previousState.gyroYaw());

        // Estimate for uncertainty in the odometry increment.
        IsotropicNoiseSE2 odometryNoise = OdometryNoise.get(twist);

        // Odometry-derived rotation increment, with uncertainty.
        VariableR1 odometryRotationIncrementRad = VariableR1.fromStdDev(
                twist.dtheta, odometryNoise.rotation());

        // Gyro drift rate during this time step.
        VariableR1 gyroBiasMeasurementRad_S = VariableR1.subtract(
                gyroIncrementRad, odometryRotationIncrementRad).times(1 / dt);

        // Fuse the previous and new bias estimates.
        VariableR1 newGyroBiasEstimateRad_S = m_gyroBiasFusor.fuse(
                previousState.gyroBias(), gyroBiasMeasurementRad_S);
        if (DEBUG)
            System.out.printf("=== gyro bias previous %s measurement %s result %s\n",
                    previousState.gyroBias(),
                    gyroBiasMeasurementRad_S,
                    newGyroBiasEstimateRad_S);

        // Gyro increment without the drift increment.
        VariableR1 correctedGyroIncrement = VariableR1.subtract(
                gyroIncrementRad, newGyroBiasEstimateRad_S.times(dt));

        // Fuse the odometry and gyro rotations.
        VariableR1 fusedRotationIncrement = m_rotationFusor.fuse(
                odometryRotationIncrementRad, correctedGyroIncrement);

        if (Experiments.INSTANCE.enabled(Experiment.PerfectGyro)) {
            // If we're trusting the gyro completely, use its verbatim increment
            // instead of the fused one.
            fusedRotationIncrement = gyroIncrementRad;
        }

        // Use the fused rotation increment with the odometry cartesian increment.
        twist = new Twist2d(twist.dx, twist.dy, fusedRotationIncrement.mean());

        // The new pose is the twist applied to the old pose.
        Pose2d newPose = previousState.state().pose().exp(twist);

        // Compute a new velocity using backward finite difference.
        VelocitySE2 velocity = VelocitySE2.velocity(previousState.state().pose(), newPose, dt);

        StateSE2 newState = new StateSE2(newPose, velocity);

        // Cartesian noise here can be zero, if we're not moving.
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(
                odometryNoise.cartesian(), fusedRotationIncrement.sigma());

        // The variance of the sum of (independent) variables is
        // just the sum of their variances. So if you drive around for
        // awhile without seeing any tags, the variance will grow
        // without bound.
        IsotropicNoiseSE2 noise = previousState.noise().plus(n0);

        m_log_prevNoise.log(() -> previousState.noise());
        m_log_updateNoise.log(() -> n0);
        m_log_newNoise.log(() -> noise);

        // The result is the new state, with verbatim measurements (to use next time).
        // There is no vision measurement here, so it's null.
        return new SwerveState(
                newState,
                noise,
                positions,
                gyroYaw,
                newGyroBiasEstimateRad_S,
                null);
    }

    /**
     * The raw gyro increment together with an estimate of its uncertainty, based
     * on white noise in the rate.
     */
    private VariableR1 getGyroMeasurementRad(double dt, Rotation2d gyroYaw, Rotation2d previousGyroYaw) {
        // Gyro raw measurement increment in this step, radians.
        double gyroStepRad = gyroYaw.minus(previousGyroYaw).getRadians();

        // Noise in the increment is the noise density times the sample time.
        // This is because the noise in the gyro step is a random walk --
        // the integral of the rate. The rate noise is uncorrelated but
        // the angle noise is not, so the angle step noise really does go to
        // zero when dt goes to zero.
        double gyroStepWhiteNoise = m_gyroWhiteNoise * Math.sqrt(dt);

        // Stddev is proportional to dt, usually the same.
        VariableR1 gyroMeasurementRad = VariableR1.fromStdDev(
                gyroStepRad, gyroStepWhiteNoise);
        return gyroMeasurementRad;
    }

    /**
     * Use the difference in module positions to determine the SE2 "twist" between
     * the previous and current poses.
     */
    private Twist2d twistFromOdometry(SwerveModulePositions positions, SwerveModulePositions previousPositions) {
        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                previousPositions, positions);
        if (!m_alwaysUpdate && Experiments.INSTANCE.enabled(Experiment.IgnoreOdometry)) {
            // Ignoring odometry means delta is always zero.
            modulePositionDelta = SwerveModuleDeltas.ZERO;
        }
        if (DEBUG) {
            System.out.printf("OdometryEstimator modulePositionDelta %s\n", modulePositionDelta);
        }
        Twist2d twist = m_kinodynamics.getKinematics().forward(modulePositionDelta);
        // Add noise, if in simulation (otherwise, this is a no-op).
        twist = m_noise.apply(twist);
        if (DEBUG) {
            System.out.printf("OdometryEstimator twist %s\n", StrUtil.twistStr(twist));
        }
        return twist;
    }

}
