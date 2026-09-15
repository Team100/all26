package org.team100.lib.gtsam.pose_estimator;

import gtsam.Key;
import gtsam.Pose2;
import gtsam.PoseRotationPrior;
import gtsam.Vector1;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Gyro {
    private final Solver estimate;
    private final shared_ptr<Diagonal> noise;

    public Gyro(Solver e) throws Throwable {
        estimate = e;
        noise = Diagonal.Sigmas(new Vector1(0.01));
    }

    /**
     * TODO: use the new planar gyro factor.
     */
    public void add(long t1_us, double yaw) throws Throwable {
        estimate.add(PoseRotationPrior.PoseRotationPriorPose2(
                Key.X(t1_us), new Pose2(0, 0, yaw), noise));
    }

}
