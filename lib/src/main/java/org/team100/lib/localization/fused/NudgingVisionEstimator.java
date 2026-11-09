package org.team100.lib.localization.fused;

import org.team100.lib.fusion.CovarianceInflation;
import org.team100.lib.fusion.Fusor;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.IsotropicNoiseSE2Logger;
import org.team100.lib.logging.LoggerFactory.LocalizerEstimateLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.VariableR1;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Updates SwerveModelHistory with any vision input, by interpolating to find a
 * pose for the vision timestamp, nudging that pose towards the vision
 * measurement, and then asking the odometry updater to replay all the later
 * odometry.
 * 
 * The "nudging" here is essentially just a weighted average; you provide the
 * weights you want at update time.
 */
public class NudgingVisionEstimator {
    private final Fusor m_cartesianFusor;
    private final Fusor m_rotationFusor;
    private final LocalizerEstimateLogger m_logEstimate;
    private final IsotropicNoiseSE2Logger m_log_prevNoise;
    private final IsotropicNoiseSE2Logger m_log_updateNoise;
    private final IsotropicNoiseSE2Logger m_log_newNoise;

    public NudgingVisionEstimator(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_logEstimate = log.localizerEstimateLogger(Level.TRACE, "estimate");
        m_log_prevNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "previous noise");
        m_log_updateNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "update noise");
        m_log_newNoise = log.isotropicNoiseSE2Logger(Level.TRACE, "new noise");
        // TODO: maybe the minimum variance here it too low?
        m_cartesianFusor = new CovarianceInflation(0.02, 0.003);
        // TODO: maybe the minimum variance here it too low?
        m_rotationFusor = new CovarianceInflation(0.02, 0.003);
    }

    /**
     * Return a state that is somewhere between the sample and the measurement.
     * 
     * The nudged state uses the nudged pose and **but the sample velocity**.
     * 
     * The nudged noise may be more, or less, than the sample noise.
     * 
     * 
     * @param sampleEstimate Interpolated history at the same time as the
     *                       measurement.
     * @param measurement    Robot pose from vision.
     * @return the nudged estimate. never null.
     */
    public LocalizerEstimate estimate(
            LocalizerEstimate sampleEstimate, NoisyPose2d measurement) {
        NoisyPose2d nudged = nudge(sampleEstimate.noisyPose(), measurement);
        LocalizerEstimate estimate = new LocalizerEstimate(
                new StateSE2(nudged.pose(), sampleEstimate.velocity()),
                nudged.noise(),
                sampleEstimate.gyroBias());
        m_logEstimate.log(() -> estimate);
        return estimate;
    }

    /**
     * Nudge the sample towards the measurement.
     * 
     * Compute the weighted average of sample and measurement, using
     * inverse-variance weighting.
     * 
     * The variance of the cartesian part is assumed to be isotropic.
     * 
     * @param sample      historical pose
     * @param measurement new (vision) input
     * @return the nudged sample. never null.
     */
    NoisyPose2d nudge(NoisyPose2d sample, NoisyPose2d measurement) {
        Pose2d samplePose = sample.pose();
        Pose2d measurementPose = measurement.pose();
        IsotropicNoiseSE2 sampleSigma = sample.noise();
        IsotropicNoiseSE2 measurementSigma = measurement.noise();

        // translation
        // the result is on the line segment between sample and measurement, so we just
        // look at the length of that segment.
        //
        // this is the start of that segment.
        VariableR1 sampleV = VariableR1.fromVariance(0, sampleSigma.cartesianVariance());
        // this is the end.
        Translation2d deltaTranslation = measurementPose.getTranslation().minus(samplePose.getTranslation());
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
            newTranslation = samplePose.getTranslation().plus(scaledTranslation);
        } else {
            // there's no angle
            newTranslation = samplePose.getTranslation();
        }

        // rotation

        Rotation2d deltaRotation = measurementPose.getRotation().minus(samplePose.getRotation());

        VariableR1 sampleRV = VariableR1.fromVariance(
                0,
                sampleSigma.rotationVariance());
        VariableR1 measurementRV = VariableR1.fromVariance(
                deltaRotation.getRadians(), measurementSigma.rotationVariance());

        VariableR1 rotation = m_rotationFusor.fuse(sampleRV, measurementRV);

        Rotation2d scaledRotation = new Rotation2d(rotation.mean());

        Rotation2d newRotation = samplePose.getRotation().plus(scaledRotation);

        Pose2d result = new Pose2d(newTranslation, newRotation);

        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromVariance(
                cartesian.variance(),
                rotation.variance());

        m_log_prevNoise.log(() -> sampleSigma);
        m_log_updateNoise.log(() -> measurementSigma);
        m_log_newNoise.log(() -> noise);

        return new NoisyPose2d(result, noise);
    }

}
