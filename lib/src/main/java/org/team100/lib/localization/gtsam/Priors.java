package org.team100.lib.localization.gtsam;

import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Priors {
    /**
     * For the pose prior, which makes the solver not freak out about indeterminacy.
     */
    private static shared_ptr<Diagonal> posePriorSigma = Diagonal.Sigmas(
            new Vector3(10, 10, 10));

    private static shared_ptr<Diagonal> biasPriorSigma = Diagonal.Sigmas(new Vector1(1));

    /**
     * Add a weak prior to help the model converge.
     */
    static void posePrior(NonlinearFactorGraph m_newFactors, Key x, Pose2 p) {
        m_newFactors.add(PriorFactor.PriorFactorPose2(x, p, posePriorSigma));
    }

    static void biasPrior(NonlinearFactorGraph m_newFactors, Key b, double bias) {
        m_newFactors.add(PriorFactor.PriorFactorDouble(b, bias, biasPriorSigma));
    }
}
