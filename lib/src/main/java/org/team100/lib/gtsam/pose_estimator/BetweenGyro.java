package org.team100.lib.gtsam.pose_estimator;

import gtsam.Key;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Rot2;
import gtsam.shared_ptr;

/** Gyro that uses PlanarGyroFactor. */
public class BetweenGyro {
    // std dev of "angle random walk" noise
    private static final double ARW_SIGMA = 1e-4;

    // std dev of bias instability
    private static final double BIAS_INSTABILITY_SIGMA = 3e-5;

    private final Solver estimate;
    private final shared_ptr<PlanarGyroParams> params;

    private double m_yaw;
    private Long t0_us = null;

    public BetweenGyro(Solver e) throws Throwable {
        estimate = e;
        params = PlanarGyroParams.makeSharedPlanarGyroParams(//
                ARW_SIGMA, BIAS_INSTABILITY_SIGMA);
    }

    /**
     * Add a PlanarGyroFactor for the pose, X(n).
     * Add PlanarGyroBiasFactor for the bias, B(n).
     * 
     * Remember to add the bias (Key.B) variable.
     */
    public void add(long t1_us, double yaw) throws Throwable {
        if (t0_us == null) {
            m_yaw = yaw;
            t0_us = t1_us;
            return;
        }
        // System.out.println("BetweenGyro.add()");

        // measurement period in seconds
        double dt = (double) (t1_us - t0_us) * 1e-6;
        // System.out.printf("dt (sec) %f\n", dt);

        // rotation between poses
        Rot2 dr = new Rot2(yaw - m_yaw);
        // dr.print("dr");

        shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRotation(//
                Key.X(t0_us), Key.X(t1_us), Key.B(t0_us), params, dr, dt);

        estimate.add(x);

        shared_ptr<PlanarGyroBiasFactor> b = PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(//
                Key.B(t0_us), Key.B(t1_us), params);

        estimate.add(b);

        m_yaw = yaw;
        t0_us = t1_us;
    }
}
