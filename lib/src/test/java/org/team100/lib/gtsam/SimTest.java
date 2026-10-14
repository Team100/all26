package org.team100.lib.gtsam;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.StrUtil;

import gtsam.BetweenFactorPose2;
import gtsam.GaussNewtonOptimizer;
import gtsam.GaussNewtonParams;
import gtsam.Key;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Rot2;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class SimTest {
    @Test
    void testSim() {
        System.out.println("========= construct sim ==========");
        Sim sim = Sim.make();
        System.out.println("========= run sim ==========");
        for (double t = 0; t < 1; t += 0.02) {
            sim.run();
            System.out.printf("t [%f] gt [%s] est [%s]\n",
                    t, StrUtil.poseStr(sim.m_groundTruthPose), sim.m_estimatedPose);
        }
        System.out.println("========= done! ==========");
    }

    @Test
    void testIndeterminate() throws Throwable {
        long t1_us = 20000;

        Key x0 = Key.X(0);
        Key x1 = Key.X(t1_us);

        Key b0 = Key.B(0);
        Key b1 = Key.B(t1_us);

        Values values = new Values();
        values.insert(x0, new Pose2());
        values.insert(x1, new Pose2());
        values.insert(b0, 0);
        values.insert(b1, 0);

        NonlinearFactorGraph factors = new NonlinearFactorGraph();
        factors.add(PriorFactor.PriorFactorPose2(x0, new Pose2(),
                Diagonal.Sigmas(new Vector3(1, 1, 1))));
        // this prior shouldn't be necessary
        factors.add(PriorFactor.PriorFactorPose2(x1, new Pose2(),
                Diagonal.Sigmas(new Vector3(10, 10, 10))));

        factors.add(PriorFactor.PriorFactorDouble(b0, 0,
                Diagonal.Sigmas(new Vector1(0.01))));
        // this prior shouldn't be necessary
        factors.add(PriorFactor.PriorFactorDouble(b1, 0,
                Diagonal.Sigmas(new Vector1(0.01))));

        // between factor of zero, so x0 should be equal to x1
        factors.add(BetweenFactorPose2.newBetweenFactorPose2(
                x0, x1, new Pose2(),
                Diagonal.Sigmas(new Vector3(0.1, 0.1, 0.1))));

        shared_ptr<PlanarGyroParams> params = PlanarGyroParams.makeSharedPlanarGyroParams(1e-4, 3e-5);
        // this should affect just the rotation between x0 and x1
        //
        //
        // this is the thing that breaks it if you use Cholesky below.
        shared_ptr<PlanarGyroFactor> gyroFactor = PlanarGyroFactor.FromRotation(
                x0, x1, b0, params, new Rot2(), 0.02);
        {
            Matrix H = new Matrix();
            Rot2 deltaR = gyroFactor.get().deltaR(0, H);
            deltaR.print("deltaR");
            H.print("deltaR H");
        }
        {
            Matrix H1 = new Matrix();
            Matrix H2 = new Matrix();
            Rot2 predict = gyroFactor.get().predict(new Rot2(), 0, H1, H2);
            predict.print("predict");
            H1.print("predict H1");
            H2.print("predict H2");
        }
        {
            Matrix H1 = new Matrix();
            Matrix H2 = new Matrix();
            Matrix H3 = new Matrix();
            double computeError = gyroFactor.get().computeError(new Rot2(), new Rot2(), 0, H1, H2, H3);
            System.out.printf("computeError: %f\n", computeError);
            H1.print("predict H1");
            H2.print("predict H2");
            H3.print("predict H3");
        }
        {
            Matrix H1 = new Matrix();
            Matrix H2 = new Matrix();
            Matrix H3 = new Matrix();
            Vector evaluateError = gyroFactor.get().evaluateError(new Pose2(), new Pose2(), 0, H1, H2, H3);
            evaluateError.print("evaluateError");
            H1.print("predict H1");
            H2.print("predict H2");
            H3.print("predict H3");
        }

        factors.add(gyroFactor);
        //
        //

        // this is literally just "between<double>" for the bias
        factors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(b0, b1, params));

        GaussNewtonParams parameters = new GaussNewtonParams();
        GaussNewtonOptimizer opt = new GaussNewtonOptimizer(factors, values, parameters);
        Values result = opt.optimize();
        result.print("result");
        // Must use QR here or it breaks at b0.
        Marginals m = Marginals.QR(factors, result);
        // this is verbose and opaque
        // m.print("marginals");
        Matrix X0cov = m.marginalCovariance(x0);
        X0cov.print("x0 covariance");
        Matrix X1cov = m.marginalCovariance(x1);
        X1cov.print("x1 covariance");
        Matrix B0cov = m.marginalCovariance(b0);
        B0cov.print("b0 covariance");
        Matrix B1cov = m.marginalCovariance(b1);
        B1cov.print("b1 covariance");
    }
}
