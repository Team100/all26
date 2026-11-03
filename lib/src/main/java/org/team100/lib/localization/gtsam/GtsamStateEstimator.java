package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.function.Supplier;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.ImmutableSwerveHistory;
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
import org.team100.lib.util.KeyBuffer;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.BatchFixedLagSmoother;
import gtsam.BetweenFactorPose2;
import gtsam.FixedLagSmoother;
import gtsam.FixedLagSmoother.Result;
import gtsam.IndeterminateSystemException;
import gtsam.Key;
import gtsam.KeyVector;
import gtsam.LevenbergMarquardtParams;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Symbol;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * State estimator that uses the GTSAM batched smoother.
 * 
 * NOTE: the actual keys use microseconds, which are not correctly
 * represented as doubles.
 * TODO: instead use the actual microseconds here.
 * 
 * TODO: oops, iterations can equal zero if there's no work to do
 * i.e. if the initial values are correct.
 */
public class GtsamStateEstimator implements StateEstimator {
    private static final boolean DEBUG = false;

    private static final double LAG_SECONDS = 0.2;

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final SwerveDriveKinematics100 m_kinematics;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    private final KeyBuffer m_keys;

    // these are not final because reset() makes new ones.
    private BatchFixedLagSmoother m_smoother;
    private NonlinearFactorGraph m_newFactors;
    private Values m_newValues;
    /** key is Key, "X(timestamp in us)", value is timestamp in seconds */
    private FixedLagSmoother.KeyTimestampMap m_newTimestamps;
    private Values m_result;

    // delete this
    // private final Key defaultKey;

    public GtsamStateEstimator(
            double initialTimestamp,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            SwerveDriveKinematics100 kinematics) {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_kinematics = kinematics;
        // the lag in seconds is compared to the max of the timestamp key map.
        m_smoother = new BatchFixedLagSmoother(
                LAG_SECONDS, new LevenbergMarquardtParams());
        m_result = new Values();
        // between updates we accumulate inputs here

        m_keys = new KeyBuffer(LAG_SECONDS);

        m_newFactors = new NonlinearFactorGraph();
        m_newValues = new Values();
        m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();

        // we keep a measurement history because some of the
        // factors are "between" factors.
        OdoGyroMeasurement initialMeasurement = new OdoGyroMeasurement(
                m_gyro.get(), positions.get());
        m_measurements = new TimeInterpolatableBuffer100<>(
                LAG_SECONDS, initialTimestamp, initialMeasurement);

        reset(0, new NoisyStateSE2(
                new StateSE2(Pose2d.kZero, VelocitySE2.ZERO),
                IsotropicNoiseSE2.high()));
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        // TODO: add timestamp duplication detection here, using KeyBuffer
        if (DEBUG)
            System.out.println("#### GtsamStateEstimator.update()");
        if (m_keys.contains(timestamp)) {
            System.out.printf("**** ERROR: skipping duplicate timestamp %f ****\n", timestamp);
            return immutableCopy();
        }
        Key x1 = poseKey(timestamp);
        // TODO: use current estimate as initial value
        m_newValues.insert(x1, new Pose2());
        m_newTimestamps.put(x1, timestamp);
        // m_newFactors.add(PriorFactor.PriorFactorPose2(x1,
        // new Pose2(0, 0, 0),
        // Diagonal.Sigmas(new Vector3(1, 1, 1))));
        odoGyro(timestamp);
        vision();

        if (DEBUG) {
            System.out.println("#### GtsamStateEstimator.update(): pending updates");
            m_newFactors.print("new factors");
            m_newValues.print("new values");

            System.out.println("#### GtsamStateEstimator.update(): current factors");

            NonlinearFactorGraph currentFactors = m_smoother.getFactors();
            currentFactors.print("current factors");

            Values currentValues = m_smoother.calculateEstimate();
            currentValues.print("current values");

            System.out.println("#### GtsamStateEstimator.update(): smoother.update");
        }
        m_keys.add(timestamp);
        try {
            Result result = m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            if (DEBUG) {
                System.out.printf("iterations %d\n", result.getIterations());
            }
        } catch (IndeterminateSystemException e) {
            System.out.println("#### IndeterminateSystemException");
            m_newFactors.print("new factors");
            m_newValues.print("new values");
        }

        if (DEBUG)
            System.out.println("#### GtsamStateEstimator.update(): smoother estimate");
        m_result = m_smoother.calculateEstimate();

        if (DEBUG)
            System.out.println("reset the accumulators");
        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

        return immutableCopy();

    }

    private Key poseKey(double timestamp) {
        return Key.X((long) (timestamp * 1e6));
    }

    private double timestamp(Symbol symbol) {
        return (double) symbol.index() / 1e6;
    }

    /**
     * Copy all the poses out of GTSAM, with stddev.
     */
    public ImmutableSwerveHistory immutableCopy() {
        if (DEBUG)
            System.out.println("#### GtsamStateEstimator.immutableCopy()");

        Marginals marginals = Marginals.QR(m_smoother.getFactors(), m_result);

        TreeMap<Double, NoisyStateSE2> list = new TreeMap<>();
        // all keys of all types known by the model
        KeyVector keys = m_result.keys();
        // extract the poses from the model
        for (int i = 0; i < keys.size(); ++i) {
            Key key = keys.at(i);
            Symbol symbol = new Symbol(key);
            if (symbol.chr() != 'x')
                continue;
            Pose2d meanPose2d = Geometry.toPose2d(m_result.atPose2(key));
            // zero velocity here
            StateSE2 state = new StateSE2(meanPose2d);
            Vector stdev = marginals.marginalCovariance(key).diagonal_cwiseSqrt();
            IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(
                    (stdev.at(0) + stdev.at(1)) / 2.0, stdev.at(2));
            NoisyStateSE2 estimate = new NoisyStateSE2(state, noise);
            list.put(timestamp(symbol), estimate);
        }
        // Infer the velocities by walking through the timestamps
        // TODO: maybe the consumer can do this if they care?
        List<Map.Entry<Double, NoisyStateSE2>> resultList = new ArrayList<>();
        Map.Entry<Double, NoisyStateSE2> prev = null;
        for (Map.Entry<Double, NoisyStateSE2> e : list.entrySet()) {
            if (prev == null) {
                prev = e;
                resultList.add(e);
                continue;
            }
            double t1 = e.getKey();
            NoisyStateSE2 s1 = e.getValue();
            double dt = t1 - prev.getKey();
            Pose2d p0 = prev.getValue().pose();
            Pose2d p1 = s1.pose();
            VelocitySE2 v1 = VelocitySE2.velocity(p0, p1, dt);
            NoisyStateSE2 s1v = new NoisyStateSE2(
                    new StateSE2(p1, v1), s1.noise());
            resultList.add(Map.entry(t1, s1v));
        }
        return ImmutableSwerveHistory.fromStates(resultList, LAG_SECONDS);
    }

    /** Add factor for odo/gyro measurement */
    void odoGyro(double timestamp) {
        if (DEBUG)
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
                poseKey(previousTimestamp),
                poseKey(timestamp),
                measurement,
                noise(twistVector));
        m_newFactors.add(newBetweenFactorPose2);

        if (DEBUG) {
            System.out.println("#### GtsamStateEstimator.odoGyro(): factor");
            newBetweenFactorPose2.get().print();
        }
        m_measurements.put(timestamp, newMeasurement);
    }

    void vision() {
        if (DEBUG)
            System.out.println("#### GtsamStateEstimator.vision()");
        for (VisionMeasurement v : m_vision.get()) {
            double visionTimestamp = v.timestamp();
            NoisyPose2d visionMeasurement = v.noisyMeasurement();
        }
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        return immutableCopy().get(timestampS);
    }

    @Override
    public void reset(double timestamp, NoisyStateSE2 state) {
        if (DEBUG)
            System.out.println("#### GtsamStateEstimator.reset()");
        m_measurements.reset(
                timestamp,
                new OdoGyroMeasurement(
                        m_gyro.get(),
                        m_positions.get()));

        // forget everything
        m_smoother = new BatchFixedLagSmoother(
                LAG_SECONDS, new LevenbergMarquardtParams());
        m_result = new Values();
        m_newFactors = new NonlinearFactorGraph();
        m_newValues = new Values();
        m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();
        m_keys.clear();

        Key x0 = poseKey(timestamp);
        Pose2d priorPose2d = state.pose();
        Pose2 prior = Geometry.toPose2(priorPose2d);
        IsotropicNoiseSE2 noise = state.noise();

        Vector3 sigmas = new Vector3(noise.cartesian(), noise.cartesian(), noise.rotation());
        shared_ptr<Diagonal> sigma = Diagonal.Sigmas(sigmas);
        m_newFactors.add(PriorFactor.PriorFactorPose2(
                x0, prior, sigma));
        m_newValues.insert(x0, prior);
        m_newTimestamps.put(x0, timestamp);

        if (DEBUG) {
            System.out.println("#### GtsamStateEstimator.reset() pending");
            m_newFactors.print("new factors");
            m_newValues.print("new values");

            System.out.println("#### GtsamStateEstimator.reset() smoother update");
        }
        m_keys.add(timestamp);
        Result result = m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
        if (DEBUG) {
            System.out.printf("iterations %d\n", result.getIterations());
            System.out.println("#### GtsamStateEstimator.reset() smoother estimate");
        }
        m_result = m_smoother.calculateEstimate();
        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return new TreeMap<>();
    }

    @Override
    public void close() {
    }

    void dump() {
        m_result.print("dump");
        m_measurements.dump();
    }

    /** Speed-dependent noise. */
    private shared_ptr<Diagonal> noise(Vector3 twist) {
        double distance = twist.norm();
        return Diagonal.Sigmas(
                new Vector3(
                        noise(distance),
                        noise(distance),
                        noise(distance)));
    }

    /** Speed-dependent noise. */
    private double noise(double distance) {
        // this used to be 0.0001 but gtsam didn't like it
        return 0.01 + 0.02 * distance;
    }

}
