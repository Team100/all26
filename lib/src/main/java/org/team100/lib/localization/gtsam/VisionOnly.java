package org.team100.lib.localization.gtsam;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.gtsam.util.Geometry;

import gtsam.BetweenFactorPose2;
import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarProjectionFactor1;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Values;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * Like vision but doesn't try to interpolate anything.
 */
public class VisionOnly {

    /** For the "between" factor, allow 0.1m between poses. */
    private static shared_ptr<Diagonal> smootherSigma = Diagonal.Sigmas(
            new Vector3(0.1, 0.1, 0.1));

    /**
     * For the pose prior, which makes the solver not freak out about indeterminacy.
     */
    private static shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(
            new Vector3(10, 10, 10));

    /**
     * TODO: move the constants here to the class, make this not static.
     */
    static void visionOnly(
            Values m_result,
            Supplier<List<Corners>> m_vision,
            NonlinearFactorGraph m_newFactors,
            Values m_newValues,
            KeyTimestampMap m_newTimestamps) {
        for (Corners v : m_vision.get()) {
            Key x = GtsamStateEstimator.poseKey(v.timestamp());
            m_newFactors.add(PlanarProjectionFactor1.newPlanarProjectionFactor1(
                    x,
                    Geometry.toPoint3(v.landmark()),
                    Geometry.toPoint2(v.measurement()),
                    Geometry.toPose3(v.offset().plus(GtsamStateEstimator.CAM)),
                    Geometry.toCal3DS2(v.intrinsic(), v.distortion()),
                    GtsamStateEstimator.m_visionNoise));
            if (m_newValues.exists(x) || m_result.exists(x)) {
                // We've already made a new value for this timestamp.
                continue;
            }
            prior(m_newFactors, m_newValues, m_newTimestamps, v.timestamp(), x);
            between(m_result, m_newFactors, x);
        }
    }

    /**
     * Add a weak prior to help the model converge.
     */
    private static void prior(NonlinearFactorGraph m_newFactors, Values m_newValues,
            KeyTimestampMap m_newTimestamps, double timestamp, Key x) {
        m_newFactors.add(PriorFactor.PriorFactorPose2(x, new Pose2(), posePriorSigma));
        m_newTimestamps.put(x, timestamp);
        m_newValues.insert(x, new Pose2());
    }

    /**
     * Add a "between" factor to smooth the pose time-series.
     */
    private static void between(Values m_result, NonlinearFactorGraph m_newFactors, Key x) {
        Key lastPoseKey = GtsamStateEstimator.lastKey(m_result, 'x');
        if (lastPoseKey != null) {
            m_newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                    lastPoseKey, x, new Pose2(), smootherSigma));
        }
    }

}
