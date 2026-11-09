package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.Collection;
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
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.LongLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.util.DiscreteTimeInterpolatableBuffer100;
import org.team100.lib.util.Math100;
import org.wpilib.framework.RobotBase;
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
import gtsam.NonlinearFactor;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Pose2;
import gtsam.Rot2;
import gtsam.Symbol;
import gtsam.Values;
import gtsam.Vector;
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
    private static final boolean DEBUG = false;

    /**
     * Using a really long lag window doesn't seem to help anything
     * and it's really slow to solve.
     */
    private static final double LAG_SECONDS = 0.2;
    private static final long LAG_US = (long) (LAG_SECONDS * 1e6);

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<Corners>> m_vision;
    private final SwerveDriveKinematics100 m_kinematics;

    private final Odometry m_odometry;

    private final DiscreteTimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    // TODO: make the gyro params much softer initially, and then tighten
    // in order to correctly calibrate a nonzero drift rate.
    private shared_ptr<PlanarGyroParams> m_params;
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
        // add noise if this is a simulation.
        m_odometry = new Odometry(
                m_kinematics, RobotBase.isSimulation(), m_gyro, m_positions);

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
        m_measurements = new DiscreteTimeInterpolatableBuffer100<>(
                LAG_US, (long) (initialTimestamp * 1e6), initialMeasurement);

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
            double timestamp = Keys.sec(symbol);
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
            double timestamp = Keys.sec(symbol);
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
        Key b1 = Keys.biasKey(timestamp);
        if (m_result.exists(x1)) {
            // skip duplicate
            if (DEBUG)
                System.out.printf("duplicate key: %s\n", x1);
            return immutableCopy();
        }

        m_odometry.odoGyro(
                m_result, m_params, timestamp, m_measurements,
                m_newFactors, m_newValues, m_newTimestamps);

        // Vision factors.
        Collection<Long> timestamps = Vision.vision(
                m_result, m_vision.get(), m_newFactors, m_newValues, m_newTimestamps);

        // Interpolated odometry required by vision factors.
        addInterpolatedFactorsAndValues(timestamps);

        try {
            if (DEBUG) {
                System.out.println("update");
                m_newFactors.print("new factors");
            }
            Result result = m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            m_log_iterations.log(result::getIterations);

        } catch (IndeterminateSystemException e) {
            System.out.printf("#### IndeterminateSystemException %s", e);
            // it can get stuck here so try just resetting
            reset(timestamp);
        }

        m_result = m_smoother.calculateEstimate();

        if (DEBUG) {
            System.out.println("### show factor errors");
            NonlinearFactorGraph factors = m_smoother.getFactors();
            // factors.printErrors(m_result, "errors");
            for (int i = 0; i < factors.size(); ++i) {
                shared_ptr<NonlinearFactor> factor = factors.at(i);
                NonlinearFactor f = factor.get();
                if (f == null)
                    continue;
                double err = f.error(m_result);
                System.out.printf("\n### factor %d err %f\n", i, err);
                f.print();
            }
        }

        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

        return immutableCopy();
    }

    /** reset to the most-recent pose, with high uncertainty. */
    public void reset(double timestamp) {
        reset(timestamp, new NoisyStateSE2(
                new StateSE2(Geometry.toPose2d(lastPose(m_result))),
                IsotropicNoiseSE2.high()));
    }

    /**
     * Add values etc as required for vision factors.
     */
    private void addInterpolatedFactorsAndValues(Collection<Long> timestamps) {
        for (long tus : timestamps) {
            // t is the timestamp required by a vision factor.

            // Pose key for timestamp t.
            Key x = Keys.poseKey(tus);
            if (m_newValues.exists(x) || m_result.exists(x)) {
                // There's already a value at this time, nothing left to do.
                continue;
            }
            // There is no value for this time.

            // We want a gyro/odo measurement bracket we can use for interpolation.
            Entry<Long, OdoGyroMeasurement> before = m_measurements.lowerEntry(tus);
            Entry<Long, OdoGyroMeasurement> after = m_measurements.higherEntry(tus);
            boolean noBefore = before == null || !m_result.exists(Keys.poseKey(before.getKey()));
            boolean noAfter = after == null || !m_result.exists(Keys.poseKey(after.getKey()));

            if (noBefore && noAfter) {
                // There are no nearby states at all.
                // This should only happen for the initial update.
                // Use zero as the initial value and prior.
                Pose2 p = new Pose2();
                m_newValues.insert(x, p);
                m_newTimestamps.put(x, (double) (tus / 1e6));
                Priors.posePrior(m_newFactors, x, p);
                continue;
            }
            if (noBefore) {
                // There is only an "after" state, so use that
                // as the initial value and prior.
                // This should only happen a few times at startup.
                long afterUs = after.getKey();
                Key x1 = Keys.poseKey(afterUs);
                Pose2 p = m_result.atPose2(x1);
                m_newValues.insert(x, p);
                m_newTimestamps.put(x, (double) (tus / 1e6));
                Priors.posePrior(m_newFactors, x, p);
                continue;
            }
            if (noAfter) {
                // There is only a "before" state, so use that
                // as the initial value and prior.
                // This should never happen: vision always lags odometry.
                long beforeUs = before.getKey();
                Key x1 = Keys.poseKey(beforeUs);
                Pose2 p = m_result.atPose2(x1);
                m_newValues.insert(x, p);
                m_newTimestamps.put(x, (double) (tus / 1e6));
                Priors.posePrior(m_newFactors, x, p);
                continue;
            }

            // There are both "before" and "after" states, so interpolate.
            long t0us = before.getKey();
            long t1us = after.getKey();
            long dtUs0 = tus - t0us;
            long dtUs1 = t1us - tus;
            long span = t1us - t0us;
            double s = (double) (dtUs0) / (double) (span);

            Key x0 = Keys.poseKey(t0us);
            Key x1 = Keys.poseKey(t1us);

            // Interpolate estimates for the initial value.
            Pose2 p0 = m_result.atPose2(x0);
            Pose2 p1 = m_result.atPose2(x1);
            Pose2 p = Geometry.interpolate(p0, p1, s);
            m_newValues.insert(x, p);
            m_newTimestamps.put(x, (double) (tus / 1e6));

            // Interpolated measurement for this timestamp
            OdoGyroMeasurement lerp = m_measurements.get(tus);
            OdoGyroMeasurement m0 = before.getValue();
            OdoGyroMeasurement m1 = after.getValue();

            // Odometry factors using the interpolated measurement.
            m_newFactors.add(m_odometry.getOdometryFactor(m_kinematics,
                    x0, x, m0.odo(), lerp.odo()));
            m_newFactors.add(m_odometry.getOdometryFactor(m_kinematics,
                    x, x1, lerp.odo(), m1.odo()));

            // Add new bias value, which is used by the factors below.
            Key b = Keys.biasKey(tus);
            Key b0 = Keys.biasKey(t0us);
            Key b1 = Keys.biasKey(t1us);

            // Interpolate estimates for the initial value.
            double bias0 = m_result.atDouble(b0);
            double bias1 = m_result.atDouble(b1);
            double bias = Math100.interpolate(bias0, bias1, s);
            m_newValues.insert(b, bias);
            m_newTimestamps.put(b, (double) (tus / 1e6));

            // Gyro factors using the interpolated measurement
            Rot2 dr0 = new Rot2(lerp.gyro().minus(m0.gyro()).getRadians());
            Rot2 dr1 = new Rot2(m1.gyro().minus(lerp.gyro()).getRadians());
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x0, x, b0, m_params, dr0, dtUs0));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x, x1, b, m_params, dr1, dtUs1));

            // Interpolate gyro bias factors.
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b0, b, m_params));
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b, b1, m_params));
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
            double timestamp = Keys.sec(symbol);
            switch (symbol.chr()) {
                case 'x' -> {
                    if (m_result.exists(key)) {
                        Pose2 x = m_result.atPose2(key);
                        Pose2d meanPose2d = Geometry.toPose2d(x);
                        // zero velocity here, fixed below.
                        StateSE2 state = new StateSE2(meanPose2d);
                        Vector stdev = marginals.marginalCovariance(key).diagonal_cwiseSqrt();
                        IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(
                                (stdev.at(0) + stdev.at(1)) / 2.0, stdev.at(2));
                        // IsotropicNoiseSE2 noise = IsotropicNoiseSE2.fromStdDev(0.1, 0.1);
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
            // if (DEBUG)
            // System.out.printf("%f\n", dt);
            Pose2d p0 = prev.getValue().pose();
            Pose2d p1 = s1.pose();
            VelocitySE2 v1 = VelocitySE2.velocity(p0, p1, dt);
            NoisyStateSE2 s1v = new NoisyStateSE2(
                    new StateSE2(p1, v1), s1.noise());
            resultList.add(Map.entry(t1, s1v));
            prev = e;
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
                (long) (timestamp * 1e6),
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
        Key b0 = Keys.biasKey(timestamp);

        Pose2d pose2dPrior = state.pose();
        Pose2 posePrior = Geometry.toPose2(pose2dPrior);

        //
        Priors.posePrior(m_newFactors, x0, posePrior);
        m_newValues.insert(x0, posePrior);
        m_newTimestamps.put(x0, timestamp);

        //
        Priors.biasPrior(m_newFactors, b0, 0.0);
        m_newValues.insert(b0, 0);
        m_newTimestamps.put(b0, timestamp);

        m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);

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
