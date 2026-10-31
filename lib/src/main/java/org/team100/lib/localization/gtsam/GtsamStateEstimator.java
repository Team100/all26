package org.team100.lib.localization.gtsam;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Supplier;

import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.NoEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.localization.fused.VisionMeasurement;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.wpilib.math.geometry.Rotation2d;

import gtsam.BatchFixedLagSmoother;
import gtsam.FixedLagSmoother;
import gtsam.LevenbergMarquardtParams;
import gtsam.NonlinearFactorGraph;
import gtsam.Values;

/**
 * State estimator that uses the GTSAM batched smoother.
 */
public class GtsamStateEstimator implements StateEstimator {

    private static final double LAG_SECONDS = 0.2;
    private static final ImmutableSwerveHistory defaultHistory = new ImmutableSwerveHistory(
            Set.of(Map.entry(0.0, new LocalizerEstimate(
                    new StateSE2(),
                    IsotropicNoiseSE2.high(),
                    VariableR1.fromStdDev(0, 1)))),
            LAG_SECONDS);

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final BatchFixedLagSmoother m_smoother;
    private final NonlinearFactorGraph m_newFactors;
    private final Values m_newValues;
    /** key is Key, "X(timestamp in us)", value is timestamp in us */
    private final FixedLagSmoother.KeyTimestampMap m_newTimestamps;
    private Values m_result;

    public GtsamStateEstimator(
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision) throws Throwable {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_smoother = new BatchFixedLagSmoother(
                LAG_SECONDS, new LevenbergMarquardtParams());
        m_result = new Values();
        // between updates we accumulate inputs here

        m_newFactors = new NonlinearFactorGraph();
        m_newValues = new Values();
        m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();
    }

    public ImmutableSwerveHistory update(double timestamp) {
        try {
            odoGyro(timestamp);
            vision();
            m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            m_result = m_smoother.calculateEstimate();
            return immutableCopy();
        } catch (Throwable e) {
            barf(e);
            return defaultHistory;
        }
    }

    private static void barf(Throwable e) {
        System.out.printf("GTSAM ERROR %s\n", e);
        e.printStackTrace();
    }

    public ImmutableSwerveHistory immutableCopy() {
        return defaultHistory;
    }

    void odoGyro(double timestamp) {
        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());
    }

    void vision() {
        for (VisionMeasurement v : m_vision.get()) {
            double visionTimestamp = v.timestamp();
            NoisyPose2d visionMeasurement = v.noisyMeasurement();
        }
    }

    /** Catches Throwable and returns NoEstimate */
    public static StateEstimator get(
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision) {
        try {
            return new GtsamStateEstimator(gyro, positions, vision);
        } catch (Throwable e) {
            barf(e);
            return new NoEstimate();
        }
    }

    public NoisyStateSE2 getState(double timestampS) {
        return new NoisyStateSE2(new StateSE2(), IsotropicNoiseSE2.high());
    }

    public void reset(double timestamp, NoisyStateSE2 state) {
    }

    public void setHeedRadiusM(double heedRadiusM) {
    }

    public Map<Double, NoisyStateSE2> all() {
        return new TreeMap<>();
    }

    public void close() {
    }
}
