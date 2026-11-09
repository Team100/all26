package org.team100.lib.localization.gtsam;

import java.util.Collection;

import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Betweens {
    /** For the "between" factor, allow 0.1m between poses. */
    private static shared_ptr<Diagonal> smootherSigma = Diagonal.Sigmas(
            new Vector3(0.1, 0.1, 0.1));

    /**
     * Add a "between" factor to smooth the pose time-series.
     */
    static void poseBetween(
            Values m_result, NonlinearFactorGraph m_newFactors, Key x) {
        Key lastPoseKey = GtsamStateEstimator.lowerKey(m_result, 'x', x);
        if (lastPoseKey != null) {
            m_newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                    lastPoseKey, x, new Pose2(), smootherSigma));
        }
    }

    /**
     * For each new timestamp, add a "between" factor from the previous state, to
     * smooth out the estimates.
     */
    static void betweens(
            Values m_result,
            Collection<Double> timestamps,
            NonlinearFactorGraph newFactors,
            Values newValues) {
        for (double timestamp : timestamps) {
            Key x = Keys.poseKey(timestamp);
            if (!newValues.exists(x) && !m_result.exists(x)) {
                Betweens.poseBetween(m_result, newFactors, x);
            }
        }
    }
}
