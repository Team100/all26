package org.team100.lib.gtsam.pose_estimator;

import gtsam.Key;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;

/**
 * A prior is a unary factor.
 * 
 * If there is no factor attached to a variable, or if only one component of a
 * variable is constrained (e.g. yaw but not x or y) the solver will fail.
 * In that case, use a very noisy prior.
 */
public class Prior {
    private final Solver estimate;

    public Prior(Solver e) {
        estimate = e;
    }

    public void add(
            Key key,
            Pose2 value,
            shared_ptr<? extends Base> noise) throws Throwable {
        estimate.add(PriorFactor.PriorFactorPose2(
                key, value, noise));
    }

    public void add(
            Key key,
            double value,
            shared_ptr<? extends Base> noise) throws Throwable {
        estimate.add(PriorFactor.PriorFactorDouble(
                key, value, noise));
    }
}
