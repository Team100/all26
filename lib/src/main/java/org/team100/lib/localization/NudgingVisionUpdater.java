package org.team100.lib.localization;

import org.team100.lib.coherence.Takt;
import org.team100.lib.fusion.CovarianceInflation;
import org.team100.lib.fusion.Fusor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.IsotropicNoiseSE2Logger;
import org.team100.lib.logging.LoggerFactory.SwerveStateLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Updates SwerveModelHistory with any vision input, by interpolating to find a
 * pose for the vision timestamp, nudging that pose towards the vision
 * measurement, and then asking the odometry updater to replay all the later
 * odometry.
 * 
 * The "nudging" here is essentially just a weighted average; you provide the
 * weights you want at update time.
 */
public class NudgingVisionUpdater implements VisionUpdater {

    private final SwerveHistory m_history;
    /** For replay. */
    private final OdometryUpdater m_odometryUpdater;
    private final Fusor m_cartesianFusor;
    private final Fusor m_rotationFusor;
    private final SwerveStateLogger m_logState;

    private final IsotropicNoiseSE2Logger m_log_prevNoise;
    private final IsotropicNoiseSE2Logger m_log_updateNoise;
    private final IsotropicNoiseSE2Logger m_log_newNoise;

    /** To measure time since last update, for indicator. */
    private double m_latestTimeS;

    public NudgingVisionUpdater(
            LoggerFactory parent,
            SwerveHistory history,
            OdometryUpdater odometryUpdater) {
        LoggerFactory log = parent.type(this);
        m_history = history;
        m_odometryUpdater = odometryUpdater;
        m_logState = log.swerveStateLogger(Level.TRACE, "state");
        m_log_prevNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "previous noise");
        m_log_updateNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "update noise");
        m_log_newNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "new noise");
        // TODO: maybe the minimum variance here it too low?
        m_cartesianFusor = new CovarianceInflation(0.02, 0.003);
        // TODO: maybe the minimum variance here it too low?
        m_rotationFusor = new CovarianceInflation(0.02, 0.003);
        m_latestTimeS = 0;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     * 
     * @param timestamp        When the measurement was made.
     * @param noisyMeasurement Robot pose from vision.
     */
    @Override
    public void put(double timestamp, NoisyPose2d noisyMeasurement) {
        // System.out.printf("vision updater visionNoise %s\n", visionNoise);

        // Skip too-old measurement
        if (m_history.tooOld(timestamp)) {
            return;
        }

        // Sample the history at the measurement time.
        SwerveState sample = m_history.getRecord(timestamp);

        // Nudge the sample towards the measurement.
        SwerveState newState = newState(sample, noisyMeasurement);

        m_logState.log(() -> newState);

        // Remember the result.
        m_history.put(timestamp, newState);

        // Replay everything after the sample.
        m_odometryUpdater.replay(timestamp);

        // Remember the time of this update.
        m_latestTimeS = Takt.get();
    }

    /**
     * Compute the new state, based on the sample.
     * 
     * Position and gyro measurements are left alone.
     */
    SwerveState newState(
            SwerveState sample, NoisyPose2d noisyMeasurement) {

        // Nudge the sample pose towards the measurement.
        ModelSE2 sampleModel = sample.state();

        NoisyPose2d noisySample = new NoisyPose2d(
                sampleModel.pose(), sample.noise());

        NoisyPose2d nudged = nudge(noisySample, noisyMeasurement);

        // Velocity is unchanged.
        ModelSE2 model = new ModelSE2(nudged.pose(), sampleModel.velocity());

        IsotropicNoiseSE2 noise = nudged.noise();

        // Odometry and gyro measurements are unchanged.
        SwerveState newState = new SwerveState(
                model,
                noise,
                sample.positions(),
                sample.gyroYaw(),
                sample.gyroBias());
        return newState;
    }

    /**
     * The age of the last pose estimate, in seconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        return Takt.get() - m_latestTimeS;
    }

    /////////////////////////////////////////

    /**
     * Compute the weighted average of sample and measurement, using
     * inverse-variance weighting.
     * 
     * The variance of the cartesian part is assumed to be isotropic.
     * 
     * @param sample      historical pose
     * @param measurement new (vision) input
     */
    NoisyPose2d nudge(
            NoisyPose2d noisySample,
            NoisyPose2d noisyMeasurement) {

        Pose2d sample = noisySample.pose();
        Pose2d measurement = noisyMeasurement.pose();
        IsotropicNoiseSE2 stateSigma = noisySample.noise();
        IsotropicNoiseSE2 measurementSigma = noisyMeasurement.noise();

        // translation
        // the result is on the line segment between sample and measurement, so we just
        // look at the length of that segment.
        //
        // this is the start of that segment.
        VariableR1 sampleV = VariableR1.fromVariance(0, stateSigma.cartesianVariance());
        // this is the end.
        Translation2d deltaTranslation = measurement.getTranslation().minus(sample.getTranslation());
        // the variance of the delta is the sum of the two variances
        double deltaNorm = deltaTranslation.getNorm();
        double deltaVariance = measurementSigma.cartesianVariance();
        VariableR1 measurementV = VariableR1.fromVariance(deltaNorm, deltaVariance);

        VariableR1 cartesian = m_cartesianFusor.fuse(sampleV, measurementV);

        Translation2d newTranslation;
        if (deltaNorm > 1e-6) {
            Rotation2d deltaTranslationDirection = deltaTranslation.getAngle();
            Translation2d scaledTranslation = new Translation2d(
                    cartesian.mean(), deltaTranslationDirection);
            newTranslation = sample.getTranslation().plus(scaledTranslation);
        } else {
            // there's no angle
            newTranslation = sample.getTranslation();
        }
        // rotation

        Rotation2d deltaRotation = measurement.getRotation().minus(sample.getRotation());

        VariableR1 sampleRV = VariableR1.fromVariance(
                0,
                stateSigma.rotationVariance());
        VariableR1 measurementRV = VariableR1.fromVariance(
                deltaRotation.getRadians(), measurementSigma.rotationVariance());

        VariableR1 rotation = m_rotationFusor.fuse(sampleRV, measurementRV);

        Rotation2d scaledRotation = new Rotation2d(rotation.mean());

        Rotation2d newRotation = sample.getRotation().plus(scaledRotation);

        Pose2d result = new Pose2d(newTranslation, newRotation);

        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromVariance(
                cartesian.variance(),
                rotation.variance());

        m_log_prevNoise.log(() -> stateSigma);
        m_log_updateNoise.log(() -> measurementSigma);
        m_log_newNoise.log(() -> noise);

        return new NoisyPose2d(result, noise);
    }

}
