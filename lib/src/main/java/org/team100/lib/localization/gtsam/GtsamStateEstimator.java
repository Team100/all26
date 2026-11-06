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
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.NoisyStateSE2;
import org.team100.lib.util.Math100;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.util.Nat;

import gtsam.BatchFixedLagSmoother;
import gtsam.BetweenFactorPose2;
import gtsam.Cal3DS2;
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
import gtsam.PlanarProjectionFactor1;
import gtsam.Point2;
import gtsam.Point3;
import gtsam.Pose2;
import gtsam.Pose3;
import gtsam.PriorFactor;
import gtsam.Rot2;
import gtsam.Symbol;
import gtsam.Values;
import gtsam.Vector2;
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
    private static final boolean DEBUG = true;

    private static final double LAG_SECONDS = 0.2;

    /**
     * The camera coordinates is z-forward, x-right, but the camera
     * "offset" is measured in x-forward, z-up coordinates (because
     * it's simpler to think about). Add this transform to get
     * the actual offset, with the z-axis pointing in the right
     * direction.
     */
    private static final Transform3d CAM = new Transform3d(
            Translation3d.kZero,
            new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(), //
                    0, 0, 1, //
                    -1, 0, 0, //
                    0, -1, 0)));

    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<Corners>> m_vision;
    private final SwerveDriveKinematics100 m_kinematics;
    private final TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements;
    // TODO: make the gyro params much softer initially, and then tighten
    // in order to correctly calibrate a nonzero drift rate.
    private final shared_ptr<PlanarGyroParams> m_params;
    private final DoubleLogger m_log_gyro_bias;

    // the camera itself is good to a pixel; the apriltag detector
    // might be good to a few pixels, depending on the lens, and the
    // lens calibration might be good to a pixel.
    // TODO: look carefully at camera accuracy
    private final shared_ptr<Diagonal> m_visionNoise;

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
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_kinematics = kinematics;
        m_visionNoise = Diagonal.Sigmas(new Vector2(2, 2));
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

    /** Returns most-recent key of type c, or null if none exists */
    private Key lastKey(char c) {
        // TODO: is it slow to do this every time, cache or keep a lookaside.
        KeyVector allkeys = m_result.keys();
        double maxTime = -Double.MAX_VALUE;
        Key maxKey = null;
        for (int i = 0; i < allkeys.size(); ++i) {
            Key key = allkeys.at(i);
            Symbol symbol = new Symbol(key);
            double timestamp = timestamp(symbol);
            if (symbol.chr() == c) {
                if (timestamp > maxTime) {
                    maxTime = timestamp;
                    maxKey = key;
                }
            }
        }
        return maxKey;
    }

    @Override
    public ImmutableSwerveHistory update(double timestamp) {
        // if (DEBUG)
        // System.out.printf("#### GtsamStateEstimator.update(%f)\n", timestamp);
        Key x1 = poseKey(timestamp);
        Key b0 = biasKey(timestamp);
        if (m_result.exists(x1)) {
            // System.out.printf("**** ERROR: skipping duplicate timestamp %f ****\n",
            // timestamp);
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

        // odoGyro(timestamp);
        // vision();
        visionOnly();
        // for now, priors
        // TODO: revive this
        // Vector3 poseSigmas = new Vector3(1, 1, 1);
        // shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(poseSigmas);
        // m_newFactors.add(PriorFactor.PriorFactorPose2(x1, new Pose2(),
        // posePriorSigma));

        // if (DEBUG) {
        // System.out.println("#### GtsamStateEstimator.update(): pending updates");
        // m_newFactors.print("new factors");
        // m_newValues.print("new values");
        // m_newTimestamps.print();

        // System.out.println("#### GtsamStateEstimator.update(): current factors");

        // NonlinearFactorGraph currentFactors = m_smoother.getFactors();
        // currentFactors.print("current factors");

        // Values currentValues = m_smoother.calculateEstimate();
        // currentValues.print("current values");

        // System.out.println("#### GtsamStateEstimator.update(): smoother.update");
        // }
        try {
            Result result = m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
            // if (DEBUG) {
            // System.out.printf("iterations %d\n", result.getIterations());
            // }
        } catch (IndeterminateSystemException e) {
            System.out.printf("#### IndeterminateSystemException %s", e);
            // it can get stuck here so try just resetting
            reset(timestamp, NoisyStateSE2.ZERO);
            // m_newFactors.print("new factors");
            // m_newValues.print("new values");
        }

        // if (DEBUG)
        // System.out.println("#### GtsamStateEstimator.update(): smoother estimate");
        m_result = m_smoother.calculateEstimate();

        // if (DEBUG)
        // System.out.println("reset the accumulators");
        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();

        return immutableCopy();

    }

    private Key poseKey(double timestamp) {
        return Key.X((long) (timestamp * 1e6));
    }

    private Key biasKey(double timestamp) {
        return Key.B((long) (timestamp * 1e6));
    }

    private double timestamp(Symbol symbol) {
        return (double) symbol.index() / 1e6;
    }

    /**
     * Copy all the poses out of GTSAM, with stddev.
     */
    public ImmutableSwerveHistory immutableCopy() {
        // if (DEBUG)
        // System.out.println("#### GtsamStateEstimator.immutableCopy() factors");
        NonlinearFactorGraph factors = m_smoother.getFactors();
        // factors.print("factors");
        // m_result.print("result");
        // if (DEBUG)
        // System.out.println("#### GtsamStateEstimator.immutableCopy() marginals");
        Marginals marginals = Marginals.QR(factors, m_result);

        TreeMap<Double, NoisyStateSE2> states = new TreeMap<>();
        TreeMap<Double, Double> biases = new TreeMap<>();
        // all keys of all types known by the model
        KeyVector keys = m_result.keys();
        // extract the poses from the model
        for (int i = 0; i < keys.size(); ++i) {
            Key key = keys.at(i);
            Symbol symbol = new Symbol(key);
            double timestamp = timestamp(symbol);
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

    /** Add factor for odo/gyro measurement */
    void odoGyro(double timestamp) {
        // if (DEBUG)
        // System.out.printf("#### GtsamStateEstimator.odoGyro(%f)\n", timestamp);
        Entry<Double, OdoGyroMeasurement> measurementEntry = m_measurements.lowerEntry(timestamp);
        if (measurementEntry == null) {
            // if (DEBUG)
            // System.out.printf("GtsamStateEstimator.odoGyro() null measurement for
            // timestamp %f\n", timestamp);
            return;
        }
        double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        double dt = timestamp - previousTimestamp;

        Key x0 = poseKey(previousTimestamp);
        Key x1 = poseKey(timestamp);

        Key b0 = biasKey(previousTimestamp);
        Key b1 = biasKey(timestamp);

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());

        m_newFactors.add(getOdometryFactor(
                x0, x1, previousMeasurement.odo(), newMeasurement.odo()));

        Rot2 dr = new Rot2(newMeasurement.gyro().minus(
                previousMeasurement.gyro()).getRadians());
        m_newFactors.add(PlanarGyroFactor.FromRotation(
                x0, x1, b0, m_params, dr, dt));

        m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                b0, b1, m_params));

        m_measurements.put(timestamp, newMeasurement);
    }

    private shared_ptr<BetweenFactorPose2> getOdometryFactor(
            Key x0,
            Key x1,
            SwerveModulePositions m0,
            SwerveModulePositions m1) {
        SwerveModuleDeltas deltas = SwerveModuleDeltas.modulePositionDelta(m0, m1);
        Twist2d twist = m_kinematics.forward(deltas);
        Vector3 twistVector = new Vector3(twist.dx, twist.dy, twist.dtheta);
        Pose2 measurement = new Pose2().expmap(twistVector);
        return BetweenFactorPose2.newBetweenFactorPose2(
                x0, x1, measurement, noise(twistVector));
    }

    void visionOnly() {
        // if (DEBUG)
        // System.out.println("#### GtsamStateEstimator.visionOnly()");
        // like vision but doesn't try to interpolate anything
        for (Corners v : m_vision.get()) {
            double timestamp = v.timestamp();
            Point3 landmark = Geometry.toPoint3(v.landmark());
            Point2 measurement = Geometry.toPoint2(v.measurement());
            Pose3 offset = Geometry.toPose3(v.offset().plus(CAM));
            Cal3DS2 calib = Geometry.toCal3DS2(v.intrinsic(), v.distortion());
            Key x = poseKey(timestamp);
            m_newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                    x,
                    landmark,
                    measurement,
                    offset,
                    calib,
                    m_visionNoise));

            if (m_newValues.exists(x) || m_result.exists(x)) {
                // We've already made a new value for this timestamp.
                // This happens anytime there's more than one tag in the same frame.
                // if (DEBUG)
                // System.out.printf("#### GtsamStateEstimator.visionOnly() skipping duplicate
                // timestamp %f\n",
                // timestamp);
                continue;
            }
            // adding a weak prior helps the model converge
            Vector3 poseSigmas = new Vector3(10, 10, 10);
            shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(poseSigmas);
            m_newFactors.add(PriorFactor.PriorFactorPose2(x, new Pose2(), posePriorSigma));
            m_newTimestamps.put(x, timestamp);
            m_newValues.insert(x, new Pose2());

            // add a "between" factor to smooth the pose time-series
            Key lastPoseKey = lastKey('x');
            if (lastPoseKey != null) {
                Vector3 smoother = new Vector3(0.1, 0.1, 0.1);
                shared_ptr<Diagonal> smootherSigma = Diagonal.Sigmas(smoother);
                // Pose2 lastPose = m_result.atPose2(lastPoseKey);
                m_newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                        lastPoseKey, x, new Pose2(), smootherSigma));
            }
        }
    }

    void vision() {
        // if (DEBUG)
        // System.out.println("#### GtsamStateEstimator.vision()");

        // look through the pending vision input
        // note there can be multiple records here with the same timestamp.
        for (Corners v : m_vision.get()) {
            double timestamp = v.timestamp();
            // if (DEBUG)
            // System.out.printf("#### GtsamStateEstimator.vision() timestamp %f\n",
            // timestamp);
            Entry<Double, OdoGyroMeasurement> before = m_measurements.lowerEntry(timestamp);
            if (before == null) {
                // if (DEBUG)
                // System.out.printf("no earlier measurement for timestamp %f\n", timestamp);
                continue;
            }
            Entry<Double, OdoGyroMeasurement> after = m_measurements.higherEntry(timestamp);
            if (after == null) {
                // if (DEBUG)
                // System.out.printf("no later measurement for timestamp %f\n", timestamp);
                continue;
            }

            Point3 landmark = Geometry.toPoint3(v.landmark());
            Point2 measurement = Geometry.toPoint2(v.measurement());
            Pose3 offset = Geometry.toPose3(v.offset().plus(CAM));
            Cal3DS2 calib = Geometry.toCal3DS2(v.intrinsic(), v.distortion());

            Key x = poseKey(timestamp);
            Key b = biasKey(timestamp);

            m_newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                    x,
                    landmark,
                    measurement,
                    offset,
                    calib,
                    m_visionNoise));

            if (m_newValues.exists(x)) {
                // We've already made a new value for this timestamp.
                // This happens anytime there's more than one tag in the same frame.
                // if (DEBUG)
                // System.out.printf("#### GtsamStateEstimator.vision() skipping duplicate
                // timestamp %f\n", timestamp);
                continue;
            }
            double beforeTime = before.getKey();
            Key x0 = poseKey(beforeTime);
            if (x.j == x0.j) {
                // if (DEBUG)
                // System.out.printf("#### GtsamStateEstimator.vision() timestamp is floor
                // %f\n", timestamp);
                // the vision timestamp happens to be the same as the floor
                continue;
            }
            double afterTime = after.getKey();
            Key x1 = poseKey(afterTime);
            if (x.j == x1.j) {
                // if (DEBUG)
                // System.out.printf("#### GtsamStateEstimator.vision() timestamp is ceiling
                // %f\n", timestamp);
                // the vision timestamp happens to be the same as the ceiling
                continue;
            }

            // make a new value in between
            // and wire up interpolated odometry factors

            Key b0 = biasKey(beforeTime);
            Key b1 = biasKey(afterTime);
            double dt0 = timestamp - beforeTime;
            double dt1 = afterTime - timestamp;
            // Interpolated measurement for this timestamp
            OdoGyroMeasurement m = m_measurements.get(timestamp);
            OdoGyroMeasurement m0 = before.getValue();
            OdoGyroMeasurement m1 = after.getValue();
            Rot2 dr0 = new Rot2(m.gyro().minus(m0.gyro()).getRadians());
            Rot2 dr1 = new Rot2(m1.gyro().minus(m.gyro()).getRadians());
            m_newFactors.add(getOdometryFactor(
                    x0, x, m0.odo(), m.odo()));
            m_newFactors.add(getOdometryFactor(
                    x, x1, m.odo(), m1.odo()));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x0, x, b0, m_params, dr0, dt0));
            m_newFactors.add(PlanarGyroFactor.FromRotation(
                    x, x1, b, m_params, dr1, dt1));

            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b0, b, m_params));
            m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                    b, b1, m_params));

            // for interpolation
            double s = dt0 / (afterTime - beforeTime);

            // interpolate the estimated pose to come up with an initial value.
            Pose2 beforePoseEstimate = m_result.atPose2(x0);
            Pose2 afterPoseEstimate = m_result.atPose2(x1);
            Pose2 poseEstimate = Geometry.interpolate(
                    beforePoseEstimate, afterPoseEstimate, s);
            m_newValues.insert(x, poseEstimate);

            // interpolate the estimated bias to come up with an initial value
            double beforeBiasEstimate = m_result.atDouble(b0);
            double afterBiasEstimate = m_result.atDouble(b1);
            double biasEstimate = Math100.interpolate(
                    beforeBiasEstimate, afterBiasEstimate, s);
            m_newValues.insert(b, biasEstimate);

            m_newTimestamps.put(x, timestamp);
            m_newTimestamps.put(b, timestamp);
        }
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

        Key x0 = poseKey(timestamp);
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

        // if (DEBUG) {
        // System.out.println("#### GtsamStateEstimator.reset() pending");
        // m_newFactors.print("new factors");
        // m_newValues.print("new values");

        // System.out.println("#### GtsamStateEstimator.reset() smoother update");
        // }
        // Result result = m_smoother.update(m_newFactors, m_newValues,
        // m_newTimestamps);
        // if (DEBUG) {
        // System.out.printf("iterations %d\n", result.getIterations());
        // System.out.println("#### GtsamStateEstimator.reset() smoother estimate");
        // }
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
        return 0.001 + 0.02 * distance;
    }

}
