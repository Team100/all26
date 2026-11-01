package org.team100.lib.localization.gtsam;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.Supplier;

import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.LocalizerEstimate;
import org.team100.lib.localization.NoEstimate;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.localization.fused.VisionMeasurement;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.BatchFixedLagSmoother;
import gtsam.BetweenFactorPose2;
import gtsam.FixedLagSmoother;
import gtsam.Key;
import gtsam.LevenbergMarquardtParams;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * State estimator that uses the GTSAM batched smoother.
 */
public class GtsamStateEstimator implements StateEstimator {
    private static final boolean DEBUG = true;

    private static final double LAG_SECONDS = 0.2;
    private static final ImmutableSwerveHistory defaultHistory = //
            new ImmutableSwerveHistory(
                    Set.of(Map.entry(0.0, new LocalizerEstimate(
                            new StateSE2(),
                            IsotropicNoiseSE2.high(),
                            VariableR1.fromStdDev(0, 1)))),
                    LAG_SECONDS);

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final SwerveDriveKinematics100 m_kinematics;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;

    // these are not final because reset() makes new ones.
    private BatchFixedLagSmoother m_smoother;
    private NonlinearFactorGraph m_newFactors;
    private Values m_newValues;
    /** key is Key, "X(timestamp in us)", value is timestamp in seconds */
    private FixedLagSmoother.KeyTimestampMap m_newTimestamps;
    private Values m_result;

    // delete this
    private final Key defaultKey;

    public GtsamStateEstimator(
            double initialTimestamp,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            SwerveDriveKinematics100 kinematics) throws Throwable {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_kinematics = kinematics;
        // the lag in seconds is compared to the max of the timestamp key map.
        m_smoother = new BatchFixedLagSmoother(
                LAG_SECONDS, new LevenbergMarquardtParams());
        m_result = new Values();
        // between updates we accumulate inputs here

        m_newFactors = new NonlinearFactorGraph();
        m_newValues = new Values();
        m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();

        // this is just to make it work at all, it should be
        // from the map of keys
        defaultKey = Key.X(0);

        m_newFactors.add(PriorFactor.PriorFactorPose2(defaultKey,
                new Pose2(0, 0, 0),
                Diagonal.Sigmas(new Vector3(100, 100, 100))));
        m_newValues.insert(defaultKey, new Pose2(0, 0, 0));
        m_newTimestamps.put(defaultKey, 0.0);

        // we keep a measurement history because some of the
        // factors are "between" factors.
        OdoGyroMeasurement initialMeasurement = new OdoGyroMeasurement(
                m_gyro.get(), positions.get());
        m_measurements = new TimeInterpolatableBuffer100<>(
                LAG_SECONDS, initialTimestamp, initialMeasurement);
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        System.out.println("#### GtsamStateEstimator.update()");
        try {
            Key x1 = Key.X((long) (timestamp * 1e6));
            // TODO: use current estimate as initial value
            m_newValues.insert(x1, new Pose2());
            m_newTimestamps.put(x1, timestamp);
            odoGyro(timestamp);
            vision();

            System.out.println("#### GtsamStateEstimator.update(): pending updates");
            m_newFactors.print("new factors");
            m_newValues.print("new values");

            System.out.println("#### GtsamStateEstimator.update(): current factors");
            NonlinearFactorGraph currentFactors = m_smoother.getFactors();
            currentFactors.print("current factors");

            Values currentValues = m_smoother.calculateEstimate();
            currentValues.print("current values");

            System.out.println("#### GtsamStateEstimator.update(): smoother.update");
            m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);

            System.out.println("#### GtsamStateEstimator.update(): smoother estimate");
            m_result = m_smoother.calculateEstimate();

            System.out.println("reset the accumulators");
            m_newFactors.resize(0);
            m_newValues.clear();
            m_newTimestamps.clear();

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

    public ImmutableSwerveHistory immutableCopy() throws Throwable {
        System.out.println("#### GtsamStateEstimator.immutableCopy()");

        System.out.println("#### GtsamStateEstimator.immutableCopy() get factors");
        NonlinearFactorGraph factors = m_smoother.getFactors();
        Pose2 mean = m_result.atPose2(defaultKey);
        Marginals m = Marginals.QR(factors, m_result);
        Matrix s = m.marginalCovariance(defaultKey);
        // this is the standard deviation of each component
        Vector cov = s.diagonal_cwiseSqrt();

        // convert it

        Pose2d meanPose2d = Geometry.toPose2d(mean);
        System.out.println("#### GtsamStateEstimator.immutableCopy() mean pose");
        System.out.println(meanPose2d);
        // TODO: velocity
        StateSE2 state = new StateSE2(meanPose2d);
        // TODO: different sigma for x and y
        IsotropicNoiseSE2 poseNoise = IsotropicNoiseSE2.fromStdDev(
                (cov.at(0) + cov.at(1)) / 2.0, cov.at(2));
        System.out.println("#### GtsamStateEstimator.immutableCopy() noise");
        System.out.println(poseNoise);

        // TODO: extract bias from GTSAM
        VariableR1 gyroBias = VariableR1.fromStdDev(0, 1);
        LocalizerEstimate estimate = new LocalizerEstimate(
                state, poseNoise, gyroBias);
        Set<Entry<Double, LocalizerEstimate>> set = Set.of(
                Map.entry(0.0, estimate));
        return new ImmutableSwerveHistory(
                set,
                LAG_SECONDS);
    }

    /** Add factor for odo/gyro measurement */
    void odoGyro(double timestamp) throws Throwable {
        System.out.println("#### GtsamStateEstimator.odoGyro()");
        Entry<Double, OdoGyroMeasurement> measurementEntry = m_measurements.lowerEntry(timestamp);
        if (measurementEntry == null) {
            if (DEBUG)
                System.out.printf("GtsamStateEstimator.odoGyro() null measurement for timestamp %f\n", timestamp);
            return;
        }
        Double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());

        SwerveModuleDeltas deltas = SwerveModuleDeltas.modulePositionDelta(
                previousMeasurement.odo(), newMeasurement.odo());

        // Tangent-space (twist) measurement.
        Twist2d twist = m_kinematics.forward(deltas);
        // Twist as a GTSAM vector.
        Vector3 twistVector = new Vector3(
                twist.dx,
                twist.dy,
                twist.dtheta);
        // Factor measurement.
        Pose2 measurement = new Pose2().expmap(twistVector);

        shared_ptr<BetweenFactorPose2> newBetweenFactorPose2 = BetweenFactorPose2.newBetweenFactorPose2(
                Key.X((long) (previousTimestamp * 1e6)),
                Key.X((long) (timestamp * 1e6)), measurement, noise(twistVector));
        m_newFactors.add(newBetweenFactorPose2);

        System.out.println("#### GtsamStateEstimator.odoGyro(): factor");
        newBetweenFactorPose2.get().print();

        m_measurements.put(timestamp, newMeasurement);
    }

    void vision() {
        System.out.println("#### GtsamStateEstimator.vision()");
        for (VisionMeasurement v : m_vision.get()) {
            double visionTimestamp = v.timestamp();
            NoisyPose2d visionMeasurement = v.noisyMeasurement();
        }
    }

    /** Catches Throwable and returns NoEstimate */
    public static StateEstimator get(
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            SwerveDriveKinematics100 kinematics) {
        try {
            return new GtsamStateEstimator(0, gyro, positions, vision, kinematics);
        } catch (Throwable e) {
            barf(e);
            return new NoEstimate();
        }
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        return new NoisyStateSE2(new StateSE2(), IsotropicNoiseSE2.high());
    }

    @Override
    public void reset(double timestamp, NoisyStateSE2 state) {
        System.out.println("#### GtsamStateEstimator.reset()");
        m_measurements.reset(
                timestamp,
                new OdoGyroMeasurement(
                        m_gyro.get(),
                        m_positions.get()));
        try {
            // forget everything
            m_smoother = new BatchFixedLagSmoother(
                    LAG_SECONDS, new LevenbergMarquardtParams());
            m_result = new Values();
            m_newFactors = new NonlinearFactorGraph();
            m_newValues = new Values();
            m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();

            // TODO: use timestamp key
            m_newFactors.add(PriorFactor.PriorFactorPose2(defaultKey,
                    new Pose2(0, 0, 0),
                    Diagonal.Sigmas(new Vector3(100, 100, 100))));
            m_newValues.insert(defaultKey, new Pose2(0, 0, 0));
            m_newTimestamps.put(defaultKey, 0.0);

            System.out.println("#### GtsamStateEstimator.reset() pending");
            m_newFactors.print("new factors");
            m_newValues.print("new values");

            System.out.println("#### GtsamStateEstimator.reset() smoother update");
            m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            System.out.println("#### GtsamStateEstimator.reset() smoother estimate");
            m_result = m_smoother.calculateEstimate();
            m_newFactors.resize(0);
            m_newValues.clear();
            m_newTimestamps.clear();
        } catch (Throwable e) {
            barf(e);
        }
    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return new TreeMap<>();
    }

    @Override
    public void close() {
    }

    void dump() throws Throwable {
        m_result.print("dump");
        m_measurements.dump();
    }

    /** Speed-dependent noise. */
    private shared_ptr<Diagonal> noise(Vector3 twist) throws Throwable {
        double distance = twist.norm();
        return Diagonal.Sigmas(
                new Vector3(
                        noise(distance),
                        noise(distance),
                        noise(distance)));
    }

    /** Speed-dependent noise. */
    private double noise(double distance) {
        return 0.0001 + 0.02 * distance;
    }

}
