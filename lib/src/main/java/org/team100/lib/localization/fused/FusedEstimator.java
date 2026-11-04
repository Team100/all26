package org.team100.lib.localization.fused;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Supplier;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Provides state estimates after updating vision and odometry, using "Fusors"
 * and replay.
 * 
 * The result is stored as an immutable copy, which is a step towards the GTSAM
 * way, where the computation is elsewhere.
 * 
 * This estimate is not self-updating (to make it easier to test). If you want
 * that, wrap it with UpdatedEstimate.
 */
public class FusedEstimator implements StateEstimator {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final OdometryEstimator m_odometryEstimator;
    private final NudgingVisionEstimator m_visionEstimator;

    private final double m_bufferDuration;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    private final TimeInterpolatableBuffer100<LocalizerEstimate> m_estimates;

    public FusedEstimator(
            double initialTimestamp,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            OdometryEstimator odometryEstimate,
            NudgingVisionEstimator visionEstimate) {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_odometryEstimator = odometryEstimate;
        m_visionEstimator = visionEstimate;
        double bufferDuration = 0.2;
        OdoGyroMeasurement initialMeasurement = new OdoGyroMeasurement(
                m_gyro.get(), positions.get());
        LocalizerEstimate initialEstimate = new LocalizerEstimate(
                new StateSE2(Pose2d.kZero, VelocitySE2.ZERO), IsotropicNoiseSE2.high(),
                VariableR1.fromVariance(0, 1));
        m_bufferDuration = bufferDuration;
        m_estimates = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialEstimate);
        m_measurements = new TimeInterpolatableBuffer100<>(
                bufferDuration, initialTimestamp, initialMeasurement);
    }

    /**
     * A copy of the whole history. Does not update.
     */
    public ImmutableSwerveHistory immutableCopy() {
        return new ImmutableSwerveHistory(m_estimates.entrySet(), m_bufferDuration);
    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        // does not update
        return immutableCopy().all();
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        // Does not update, in order to make testing easier.
        ImmutableSwerveHistory history = immutableCopy();
        final NoisyStateSE2 state;
        if (Experiments.INSTANCE.enabled(Experiment.ImputeVelocity)) {
            // Use consecutive poses
            NoisyStateSE2 state0 = history.get(timestampS - DT);
            NoisyStateSE2 state1 = history.get(timestampS);
            VelocitySE2 v = VelocitySE2.velocity(
                    state0.pose(),
                    state1.pose(), DT);
            state = new NoisyStateSE2(
                    new StateSE2(state1.pose(), v), state1.noise());
        } else {
            // Use the history value
            state = history.get(timestampS);
        }
        if (DEBUG) {
            System.out.printf("FusedEstimator.getState(): %s\n", state);
        }
        return state;
    }

    @Override
    public void reset(double timestampS, NoisyStateSE2 state) {
        // Always reset both at once, so they stay in sync.
        m_measurements.reset(
                timestampS,
                new OdoGyroMeasurement(
                        m_gyro.get(),
                        m_positions.get()));
        m_estimates.reset(
                timestampS,
                new LocalizerEstimate(
                        state.state(),
                        state.noise(),
                        VariableR1.fromVariance(0, 1)));
    }

    @Override
    public void close() {
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        odoGyro(timestamp);
        vision();
        return immutableCopy();
    }

    /** Print the buffer contents, for testing. */
    void dump() {
        m_estimates.dump();
        m_measurements.dump();
    }

    /**
     * Add the one-and-only odo/gyro measurement for this timestamp.
     */
    private void odoGyro(double timestamp) {
        Entry<Double, LocalizerEstimate> estimateEntry = m_estimates.lowerEntry(timestamp);
        if (estimateEntry == null) {
            if (DEBUG)
                System.out.printf("FusedEstimator.odoGyro() null estimate for timestamp %f\n", timestamp);
            return;
        }
        LocalizerEstimate previousEstimate = estimateEntry.getValue();

        Entry<Double, OdoGyroMeasurement> measurementEntry = m_measurements.lowerEntry(timestamp);
        if (measurementEntry == null) {
            if (DEBUG)
                System.out.printf("FusedEstimator.odoGyro() null measurement for timestamp %f\n", timestamp);
            return;
        }
        Double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());
        LocalizerEstimate newEstimate = m_odometryEstimator.estimate(
                previousTimestamp,
                previousEstimate,
                previousMeasurement,
                timestamp,
                newMeasurement);
        if (DEBUG)
            System.out.printf("FusedEstimator.odoGyro() newEstimate %s\n", newEstimate);
        put(timestamp, newEstimate, newMeasurement);
    }

    /**
     * Apply pending vision updates, with replay.
     */
    private void vision() {
        for (VisionMeasurement v : m_vision.get()) {
            double visionTimestamp = v.timestamp();
            if (m_measurements.tooOld(visionTimestamp))
                continue;
            NoisyPose2d visionMeasurement = v.noisyMeasurement();
            OdoGyroMeasurement sampleMeasurement = m_measurements.get(visionTimestamp);
            LocalizerEstimate sampleEstimate = m_estimates.get(visionTimestamp);
            LocalizerEstimate newEstimate = m_visionEstimator.estimate(
                    sampleEstimate, visionMeasurement);
            put(visionTimestamp, newEstimate, sampleMeasurement);
            replay(visionTimestamp);
        }
    }

    private void put(
            double timestamp,
            LocalizerEstimate estimate,
            OdoGyroMeasurement measurement) {
        // Always put both at once, so that the keys are always the same.
        m_estimates.put(timestamp, estimate);
        m_measurements.put(timestamp, measurement);
    }

    private void replay(double sampleTime) {
        if (DEBUG)
            System.out.printf("==== REPLAY FOR TIME %f\n", sampleTime);
        // Note the exclusive tailmap: we don't see the entry at timestamp.
        Set<Entry<Double, OdoGyroMeasurement>> entriesAfterSampleTime = m_measurements.tailMap(sampleTime, false)
                .entrySet();
        for (Map.Entry<Double, OdoGyroMeasurement> entry : entriesAfterSampleTime) {
            double timestamp = entry.getKey();

            Entry<Double, LocalizerEstimate> estimateEntry = m_estimates.lowerEntry(timestamp);
            if (estimateEntry == null)
                continue;
            LocalizerEstimate previousEstimate = estimateEntry.getValue();

            Entry<Double, OdoGyroMeasurement> measurementEntry = m_measurements.lowerEntry(timestamp);
            if (measurementEntry == null)
                continue;
            double previousTimestamp = measurementEntry.getKey();
            OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

            OdoGyroMeasurement newMeasurement = entry.getValue();

            LocalizerEstimate newEstimate = m_odometryEstimator.estimate(
                    previousTimestamp,
                    previousEstimate,
                    previousMeasurement,
                    timestamp,
                    newMeasurement);
            put(timestamp, newEstimate, newMeasurement);
        }
        if (DEBUG)
            System.out.printf("==== DONE REPLAYING FOR TIME %f\n", sampleTime);
    }

}
