package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.function.Supplier;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.localization.ImmutableSwerveHistory;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.StateEstimator;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.LongLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import gtsam.BatchFixedLagSmoother;
import gtsam.FixedLagSmoother;
import gtsam.FixedLagSmoother.Result;
import gtsam.IndeterminateSystemException;
import gtsam.Key;
import gtsam.KeyVector;
import gtsam.LevenbergMarquardtParams;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Pose2;
import gtsam.Rot2;
import gtsam.Symbol;
import gtsam.Values;
import gtsam.shared_ptr;

/**
 * State estimator that uses the GTSAM batched smoother.
 * 
 * The timestamp is double seconds, generally less than 1000.
 * 
 * The key field is long microseconds.
 * 
 * So we need accurate double->long with about 9 places of precision.
 * 
 * Doubles have about 15 places of precision, enough for about
 * 30 years of runtime.
 */
public class GtsamStateEstimator implements StateEstimator {
    private static final boolean DEBUG = true;

    private static final double LAG_SECONDS = 0.2;

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<Corners>> m_vision;
    private final SwerveDriveKinematics100 m_kinematics;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    // TODO: make the gyro params much softer initially, and then tighten
    // in order to correctly calibrate a nonzero drift rate.
    private final shared_ptr<PlanarGyroParams> m_params;
    private final DoubleLogger m_log_gyro_bias;
    private final LongLogger m_log_iterations;

    // these are not final because reset() makes new ones.
    private BatchFixedLagSmoother m_smoother;
    private NonlinearFactorGraph m_newFactors;
    private Values m_newValues;
    /** key is Key, "X(timestamp in us)", value is timestamp in seconds */
    private FixedLagSmoother.KeyTimestampMap m_newTimestamps;
    private Values m_result;

    public GtsamStateEstimator(
            LoggerFactory parent,
            double initialTimestamp,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<Corners>> vision,
            SwerveDriveKinematics100 kinematics,
            double whiteNoise,
            double biasNoise) {
        LoggerFactory log = parent.type(this);
        m_log_gyro_bias = log.doubleLogger(Level.TRACE, "gyro bias");
        m_log_iterations = log.longLogger(Level.TRACE, "iterations");
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_kinematics = kinematics;
        m_params = PlanarGyroParams.makeSharedPlanarGyroParams(
                whiteNoise, biasNoise);
        // the lag in seconds is compared to the max of the timestamp key map.
        m_smoother = new BatchFixedLagSmoother(
                LAG_SECONDS, new LevenbergMarquardtParams());
        m_result = new Values();
        // between updates we accumulate inputs here

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

    /** Returns most-recent key of type c, or null if none exists. */
    static Key lastKey(Values m_result, char c) {
        // TODO: is it slow to do this every time, cache or keep a lookaside.
        KeyVector allkeys = m_result.keys();
        double maxTime = -Double.MAX_VALUE;
        Key maxKey = null;
        for (int i = 0; i < allkeys.size(); ++i) {
            Key key = allkeys.at(i);
            Symbol symbol = new Symbol(key);
            double timestamp = Keys.timestamp(symbol);
            if (symbol.chr() == c) {
                if (timestamp > maxTime) {
                    maxTime = timestamp;
                    maxKey = key;
                }
            }
        }
        return maxKey;
    }

    /**
     * Return most-recent key of type c strictly less than k, or null if none
     * exists.
     */
    static Key lowerKey(Values m_result, char c, Key k) {
        // TODO: is it slow to do this every time, cache or keep a lookaside.
        KeyVector allkeys = m_result.keys();
        double maxTime = -Double.MAX_VALUE;
        Key maxKey = null;
        for (int i = 0; i < allkeys.size(); ++i) {
            Key key = allkeys.at(i);
            Symbol symbol = new Symbol(key);
            double timestamp = Keys.timestamp(symbol);
            if (key.j >= k.j)
                continue;
            if (symbol.chr() == c) {
                if (timestamp > maxTime) {
                    maxTime = timestamp;
                    maxKey = key;
                }
            }
        }
        return maxKey;
    }

    static Pose2 lastPose(Values m_result) {
        Key x = lastKey(m_result, 'x');
        if (x == null)
            return new Pose2();
        return m_result.atPose2(x);
    }

    static double lastBias(Values m_result) {
        Key x = lastKey(m_result, 'b');
        if (x == null)
            return 0;
        return m_result.atDouble(x);
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        Key x1 = Keys.poseKey(timestamp);
        Key b0 = Keys.biasKey(timestamp);
        if (m_result.exists(x1)) {
            // skip duplicate
            return immutableCopy();
        }

        // For each update, we make a new pose value and a new bias value,
        // using the latest estimate as the initial value.
        // TODO: revive this
        // Key lastPoseKey = lastKey('x');
        // Pose2 lastPose = m_result.atPose2(lastPoseKey);
        // Pose2 lastPose = new Pose2();
        // m_newValues.insert(x1, lastPose);
        // m_newTimestamps.put(x1, timestamp);

        // for now, skip the bias
        // TODO: turn this back on
        // Key lastBiasKey = lastKey('b');
        // double lastBias = m_result.atDouble(lastBiasKey);
        // m_newValues.insert(b0, lastBias);
        // m_newTimestamps.put(b0, timestamp);

        Collection<Double> timestamps = new TreeSet<>();

        // timestamps.addAll(Odometry.odoGyro(
        // m_kinematics, m_params, timestamp, m_measurements,
        // m_gyro, m_positions, m_newFactors));
        // for (double t : timestamps) {
        // Key x = Keys.poseKey(t);
        // m_newValues.insert(x, lastPose(m_result));
        // m_newTimestamps.put(x, t);
        // Key b = Keys.biasKey(t);
        // m_newTimestamps.put(b, timestamp);
        // m_newValues.insert(b, lastBias(m_result));
        // }
        // timestamps.clear();

        // add vision factors, new values, and new timestamps.
        timestamps.addAll(Vision.vision(
                m_result, m_vision.get(), m_newFactors, m_newValues, m_newTimestamps));

        // interpolated odometry
        // addInterpolatedFactorsAndValues(timestamps);

        // betweens and priors.
        betweens(timestamps);
        priors(timestamps);

        // for now, priors
        // TODO: revive this
        // Vector3 poseSigmas = new Vector3(1, 1, 1);
        // shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(poseSigmas);
        // m_newFactors.add(PriorFactor.PriorFactorPose2(x1, new Pose2(),
        // posePriorSigma));

        try {
            Result result = m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            m_log_iterations.log(result::getIterations);

        } catch (IndeterminateSystemException e) {
            System.out.printf("#### IndeterminateSystemException %s", e);
            // it can get stuck here so try just resetting
            reset(timestamp, new NoisyStateSE2(
                    new StateSE2(Geometry.toPose2d(lastPose(m_result))),
                    IsotropicNoiseSE2.high()));
        }

        m_result = m_smoother.calculateEstimate();

        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

        return immutableCopy();
    }

    /**
     * Add values, timestamps, and interpolated factors for this state and the
     * time-aligned gyro bias.
     */
    private void addInterpolatedFactorsAndValues(Collection<Double> timestamps) {
        for (double timestamp : timestamps) {
            Key x = Keys.poseKey(timestamp);
            Key b = Keys.biasKey(timestamp);

            if (m_newValues.exists(x) || m_result.exists(x)) {
                // We've already made a new value for this timestamp.
                continue;
            }
            // state for the timestamp, we use this regardless.
            // TODO: improve initial values.
            m_newTimestamps.put(x, timestamp);
            m_newValues.insert(x, new Pose2());

            Entry<Double, OdoGyroMeasurement> before = m_measurements.lowerEntry(timestamp);
            if (before == null) {
                // there's no way to interpolate
                continue;
            }
            Entry<Double, OdoGyroMeasurement> after = m_measurements.higherEntry(timestamp);
            if (after == null) {
                // there's no way to interpolate
                continue;
            }

            double beforeTime = before.getKey();
            Key x0 = Keys.poseKey(beforeTime);
            if (x.j == x0.j) {
                // the vision timestamp happens to be the same as the floor,
                // so there's nothing more required.
                continue;
            }
            double afterTime = after.getKey();
            Key x1 = Keys.poseKey(afterTime);
            if (x.j == x1.j) {
                // the vision timestamp happens to be the same as the ceiling,
                // so there's nothing more required.
                continue;
            }

            // if we got here, we're going to interpolate.

            m_newTimestamps.put(b, timestamp);
            m_newValues.insert(b, 0);

            // make a new value in between
            // and wire up interpolated odometry factors

            // Interpolated measurement for this timestamp
            OdoGyroMeasurement m = m_measurements.get(timestamp);
            OdoGyroMeasurement m0 = before.getValue();
            OdoGyroMeasurement m1 = after.getValue();

            // interpolated odometry factors
            m_newFactors.add(Odometry.getOdometryFactor(m_kinematics,
                    x0, x, m0.odo(), m.odo()));
            m_newFactors.add(Odometry.getOdometryFactor(m_kinematics,
                    x, x1, m.odo(), m1.odo()));

            // interpolated gyro factors
            Key b0 = Keys.biasKey(beforeTime);
            Key b1 = Keys.biasKey(afterTime);
            Rot2 dr0 = new Rot2(m.gyro().minus(m0.gyro()).getRadians());
            Rot2 dr1 = new Rot2(m1.gyro().minus(m.gyro()).getRadians());
            double dt0 = timestamp - beforeTime;
            double dt1 = afterTime - timestamp;
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x0, x, b0, m_params, dr0, dt0));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x, x1, b, m_params, dr1, dt1));

            // interpolated gyro bias factors
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b0, b, m_params));
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b, b1, m_params));
        }
    }

    /**
     * For each new timestamp, add a "between" factor from the previous state, to
     * smooth out the estimates.
     */
    private void betweens(Collection<Double> timestamps) {
        for (double timestamp : timestamps) {
            Key x = Keys.poseKey(timestamp);
            if (!m_newValues.exists(x) && !m_result.exists(x)) {
                Betweens.poseBetween(m_result, m_newFactors, x);
            }
        }
    }

    /**
     * For each new timestamp, add a prior.
     */
    private void priors(Collection<Double> timestamps) {
        for (double timestamp : timestamps) {
            Key x = Keys.poseKey(timestamp);
            if (!m_newValues.exists(x) && !m_result.exists(x)) {
                Priors.posePrior(m_newFactors, x);
            }
        }
    }

    /**
     * Copy all the poses out of GTSAM, with stddev.
     */
    public ImmutableSwerveHistory immutableCopy() {
        NonlinearFactorGraph factors = m_smoother.getFactors();

        // sometimes the marginals part doesn't work.
        // TODO: fix that
        Marginals marginals = Marginals.QR(factors, m_result);

        TreeMap<Double, NoisyStateSE2> states = new TreeMap<>();
        TreeMap<Double, Double> biases = new TreeMap<>();
        // all keys of all types known by the model
        KeyVector keys = m_result.keys();
        // extract the poses from the model
        for (int i = 0; i < keys.size(); ++i) {
            Key key = keys.at(i);
            Symbol symbol = new Symbol(key);
            double timestamp = Keys.timestamp(symbol);
            switch (symbol.chr()) {
                case 'x' -> {
                    if (m_result.exists(key)) {
                        Pose2 x = m_result.atPose2(key);
                        Pose2d meanPose2d = Geometry.toPose2d(x);
                        // zero velocity here
                        StateSE2 state = new StateSE2(meanPose2d);
                        // TODO: revive this
                        // Vector stdev = marginals.marginalCovariance(key).diagonal_cwiseSqrt();
                        // IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(
                        // (stdev.at(0) + stdev.at(1)) / 2.0, stdev.at(2));
                        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(0.1, 0.1);
                        NoisyStateSE2 estimate = new NoisyStateSE2(state, noise);
                        states.put(timestamp, estimate);
                    }
                }
                case 'b' -> {
                    if (m_result.exists(key)) {
                        double b = m_result.atDouble(key);
                        biases.put(timestamp, b);
                    }
                }
                default -> {
                    continue;
                }
            }
        }
        // Log the most-recent bias.
        Entry<Double, Double> lastBias = biases.lastEntry();
        if (lastBias != null) {
            m_log_gyro_bias.log(() -> lastBias.getValue());
        }
        // Infer the velocities by walking through the timestamps
        // TODO: maybe the consumer can do this if they care?
        List<Map.Entry<Double, NoisyStateSE2>> resultList = new ArrayList<>();
        Map.Entry<Double, NoisyStateSE2> prev = null;
        for (Map.Entry<Double, NoisyStateSE2> e : states.entrySet()) {
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
        if (resultList.isEmpty()) {
            NoisyStateSE2 state = new NoisyStateSE2(new StateSE2(), IsotropicNoiseSE2.high());
            return ImmutableSwerveHistory.fromStates(
                    List.of(Map.entry(0.0, state)),
                    LAG_SECONDS);
        }
        return ImmutableSwerveHistory.fromStates(resultList, LAG_SECONDS);
    }

    @Override
    public NoisyStateSE2 getState(double timestampS) {
        return immutableCopy().get(timestampS);
    }

    @Override
    public void reset(double timestamp, NoisyStateSE2 state) {
        if (DEBUG)
            System.out.printf("#### GtsamStateEstimator.reset() %f\n", timestamp);
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

        Key x0 = Keys.poseKey(timestamp);
        Pose2d pose2dPrior = state.pose();
        Pose2 posePrior = Geometry.toPose2(pose2dPrior);
        IsotropicNoiseSE2 poseNoise = state.noise();

        // avoid odometry-timestamped poses (since they're always zero)
        // TODO: put this back
        // Vector3 poseSigmas = new Vector3(poseNoise.cartesian(),
        // poseNoise.cartesian(), poseNoise.rotation());
        // shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(poseSigmas);
        // m_newFactors.add(PriorFactor.PriorFactorPose2(x0, posePrior,
        // posePriorSigma));
        // m_newValues.insert(x0, posePrior);
        // m_newTimestamps.put(x0, timestamp);

        // for now, remove this
        // TODO: put it back
        // Key b0 = biasKey(timestamp);
        // shared_ptr<Diagonal> biasPriorSigma = Diagonal.Sigmas(new Vector1(1));
        // m_newFactors.add(PriorFactor.PriorFactorDouble(
        // b0, 0, biasPriorSigma));
        // m_newValues.insert(b0, 0);
        // m_newTimestamps.put(b0, timestamp);

        // Result result = m_smoother.update(m_newFactors, m_newValues,
        // m_newTimestamps);

        m_result = m_smoother.calculateEstimate();
        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

    }

    @Override
    public Map<Double, NoisyStateSE2> all() {
        return immutableCopy().all();
    }

    @Override
    public void close() {
    }

    void dump() {
        m_result.print("dump");
        m_measurements.dump();
    }

}
