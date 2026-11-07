package org.team100.lib.localization.gtsam;

import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Priors {
    /**
     * For the pose prior, which makes the solver not freak out about indeterminacy.
     */
    private static shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(
            new Vector3(10, 10, 10));

    /**
     * Add a weak prior to help the model converge.
     */
    static void posePrior(NonlinearFactorGraph m_newFactors, Key x) {
        m_newFactors.add(PriorFactor.PriorFactorPose2(x, new Pose2(), posePriorSigma));
    }
}
