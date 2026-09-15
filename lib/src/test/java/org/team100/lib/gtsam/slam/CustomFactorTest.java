package org.team100.lib.gtsam.slam;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import gtsam.BetweenFactorPose2;
import gtsam.CustomErrorFunction;
import gtsam.CustomFactor;
import gtsam.GaussianFactor;
import gtsam.JacobianVector;
import gtsam.Key;
import gtsam.KeyVector;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.LevenbergMarquardtParams;
import gtsam.Matrix;
import gtsam.Matrix3;
import gtsam.NonlinearFactorGraph;
import gtsam.Pair;
import gtsam.Pose2;
import gtsam.PriorFactor;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Unit;
import org.team100.lib.gtsam.util.TestUtil;

/** See python/gtsam/tests/test_custom_factor.py */
public class CustomFactorTest {

    /** Test the creation of a new CustomFactor */
    @Test
    void testNew() throws Throwable {
        CustomErrorFunction error_func = new CustomErrorFunction() {
            /** Minimal error function stub */
            @Override
            public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                return new Vector(new double[] { 1, 0, 0 });
            }
        };
        shared_ptr<Unit> noise_model = Unit.Create(3);
        KeyVector keys = new KeyVector(new Key(0));
        shared_ptr<CustomFactor> cf = CustomFactor.newCustomFactor(noise_model, keys, error_func);
    }

    /** Test if calling the factor works (only error) */
    @Test
    void testCall() throws Throwable {
        Pose2 expected_pose = new Pose2(1, 1, 0);
        CustomErrorFunction error_func = new CustomErrorFunction() {
            /** Minimal error function with no Jacobian */
            @Override
            public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                Key key0 = factor.keys().at(0);
                Vector3 error = v.atPose2(key0).local(expected_pose).times(-1);
                return new Vector(error);
            }
        };
        shared_ptr<Unit> noise_model = Unit.Create(3);
        KeyVector keys = new KeyVector(new Key(0));
        shared_ptr<CustomFactor> cf = CustomFactor.newCustomFactor(noise_model, keys, error_func);
        Values v = new Values();
        v.insert(new Key(0), new Pose2(1, 0, 0));
        double e = cf.get().error(v);
        assertEquals(e, 0.5);
    }

    /** Tests if the factor result matches the GTSAM Pose2 unit test */
    @Test
    void testJacobian() throws Throwable {
        Pose2 gT1 = new Pose2(1, 2, Math.PI / 2);
        Pose2 gT2 = new Pose2(-1, 4, Math.PI);
        Pose2 expected = new Pose2(2, 2, Math.PI / 2);
        CustomErrorFunction error_func = new CustomErrorFunction() {
            /**
             * the custom error function. One can freely use variables captured
             * from the outside scope. Or the variables can be acquired by indexing `v`.
             * Jacobian is passed by modifying the H array of numpy matrices.
             */
            @Override
            public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                Key key0 = factor.keys().at(0);
                Key key1 = factor.keys().at(1);
                Pose2 gT1 = v.atPose2(key0);
                Pose2 gT2 = v.atPose2(key1);
                Vector error = new Vector(expected.local(gT1.between(gT2)));
                if (H != null) {
                    Pose2 result = gT1.between(gT2);
                    H.insert(0, result.inverse().AdjointMap().times(-1));
                    H.insert(1, Matrix3.identity());
                }
                return error;
            }
        };
        shared_ptr<Unit> noise_model = Unit.Create(3);
        KeyVector keys = new KeyVector(new Key(0), new Key(1));
        shared_ptr<CustomFactor> cf = CustomFactor.newCustomFactor(noise_model, keys, error_func);
        Values v = new Values();
        v.insert(new Key(0), gT1);
        v.insert(new Key(1), gT2);

        double e = cf.get().error(v);

        shared_ptr<BetweenFactorPose2> bf = BetweenFactorPose2.newBetweenFactorPose2(
                new Key(0), new Key(1), expected, noise_model);
        shared_ptr<GaussianFactor> gf = cf.get().linearize(v);
        shared_ptr<GaussianFactor> gf_b = bf.get().linearize(v);
        Pair<Matrix, Vector> gfj = gf.get().jacobian();
        Pair<Matrix, Vector> gf_bj = gf_b.get().jacobian();
        Matrix J_cf = gfj.first;
        Vector b_cf = gfj.second;
        Matrix J_bf = gf_bj.first;
        Vector b_bf = gf_bj.second;
        TestUtil.assertAllClose(J_cf, J_bf, 1e-6);
        TestUtil.assertAllClose(b_cf, b_bf, 1e-6);
    }

    /** Tests that we will not calculate the Jacobian if not requested */
    @Test
    void testNoJacobian() throws Throwable {
        Pose2 gT1 = new Pose2(1, 2, Math.PI / 2);
        Pose2 gT2 = new Pose2(-1, 4, Math.PI);
        Pose2 expected = new Pose2(2, 2, Math.PI / 2);
        CustomErrorFunction error_func = new CustomErrorFunction() {
            /**
             * Error function that mimics a BetweenFactor
             * 
             * @param factor reference to the current CustomFactor being evaluated
             * @param v      Values object
             * @param H      list of references to the Jacobian arrays
             * @return the non-linear error
             */
            @Override
            public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                Key key0 = factor.keys().at(0);
                Key key1 = factor.keys().at(1);
                Pose2 gT1 = v.atPose2(key0);
                Pose2 gT2 = v.atPose2(key1);
                Vector error = new Vector(expected.local(gT1.between(gT2)));
                assertNull(H);// Should be null if we only request the error
                return error;
            }
        };

        shared_ptr<Unit> noise_model = Unit.Create(3);
        KeyVector keys = new KeyVector(new Key(0), new Key(1));
        shared_ptr<CustomFactor> cf = CustomFactor.newCustomFactor(noise_model, keys, error_func);
        Values v = new Values();
        v.insert(new Key(0), gT1);
        v.insert(new Key(1), gT2);
        shared_ptr<BetweenFactorPose2> bf = BetweenFactorPose2.newBetweenFactorPose2(
                new Key(0), new Key(1), expected, noise_model);
        double e_cf = cf.get().error(v);
        double e_bf = bf.get().error(v);
        assertEquals(e_cf, e_bf, 1e-6);
    }

    /** Tests if a factor graph with a CustomFactor can be properly optimized */
    @Test
    void testOptimization() throws Throwable {
        Pose2 gT1 = new Pose2(1, 2, Math.PI / 2);
        Pose2 gT2 = new Pose2(-1, 4, Math.PI);
        Pose2 expected = new Pose2(2, 2, Math.PI / 2);

        CustomErrorFunction error_func = new CustomErrorFunction() {
            /**
             * Error function that mimics a BetweenFactor
             * 
             * @param factor reference to the current CustomFactor being evaluated
             * @param v      Values object
             * @param H      list of references to the Jacobian arrays
             * @return the non-linear error
             */
            @Override
            public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                Key key0 = factor.keys().at(0);
                Key key1 = factor.keys().at(1);
                Pose2 gT1 = v.atPose2(key0);
                Pose2 gT2 = v.atPose2(key1);
                Vector error = new Vector(expected.local(gT1.between(gT2)));
                if (H != null) {
                    Pose2 result = gT1.between(gT2);
                    H.insert(0, result.inverse().AdjointMap().times(-1));
                    H.insert(1, Matrix3.identity());
                }
                return error;
            }
        };
        shared_ptr<Unit> noise_model = Unit.Create(3);
        KeyVector keys = new KeyVector(new Key(0), new Key(1));
        shared_ptr<CustomFactor> cf = CustomFactor.newCustomFactor(
                noise_model, keys, error_func);
        NonlinearFactorGraph fg = new NonlinearFactorGraph();
        fg.add(cf);
        fg.add(PriorFactor.PriorFactorPose2(new Key(0), gT1, noise_model));
        Values v = new Values();
        v.insert(new Key(0), new Pose2(0, 0, 0));
        v.insert(new Key(1), new Pose2(0, 0, 0));
        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(fg, v, params);
        Values result = optimizer.optimize();
        TestUtil.assertPose2Equals(result.atPose2(new Key(0)), gT1, 1e-5);
        TestUtil.assertPose2Equals(result.atPose2(new Key(1)), gT2, 1e-5);
    }

    /**
     * CustomFactor that is like a BetweenFactor.
     * 
     * The java custom factor solution is about 75% as fast as the C++ "between"
     * factor.
     * 
     * On my machine for 10000 values this takes 2.0 sec.
     * 
     * It took 1.8 sec before I added the vector copy in the error function binder.
     * 
     * TODO: find a way to remove that copy.
     */
    @Test
    void testMultiOptimization() throws Throwable {
        shared_ptr<Unit> noise_model = Unit.Create(3);

        // Expectation is always the same: each pose steps 1m +x.
        Pose2 expected = new Pose2(1, 0, 0);

        NonlinearFactorGraph factorGraph = new NonlinearFactorGraph();
        // prior for initial pose is origin
        factorGraph.add(PriorFactor.PriorFactorPose2(
                new Key(0),
                new Pose2(),
                Diagonal.Sigmas(new Vector3(0.001, 0.001, 0.001))));

        // Create variables with bad initial values.
        int N = 10000;
        Values initialValues = new Values();
        for (int i = 0; i < N; ++i) {
            initialValues.insert(new Key(i), new Pose2(0, 0, 0));
            if (i > 0) {
                // since the expectation is always the same, this *could* be the
                // same function every time, but that's not a realistic situation.
                CustomErrorFunction error_func = new CustomErrorFunction() {
                    @Override
                    public Vector apply(CustomFactor factor, Values v, JacobianVector H) throws Throwable {
                        Pose2 gT1 = v.atPose2(factor.keys().at(0));
                        Pose2 gT2 = v.atPose2(factor.keys().at(1));
                        Pose2 between = gT1.between(gT2);
                        Vector error = new Vector(expected.local(between));
                        if (H != null) {
                            H.insert(0, between.inverse().AdjointMap().times(-1));
                            H.insert(1, Matrix3.identity());
                        }
                        return error;
                    }
                };
                factorGraph.add(CustomFactor.newCustomFactor(
                        noise_model,
                        new KeyVector(new Key(i - 1), new Key(i)),
                        error_func));
            }
        }

        long t0_ns = System.nanoTime();

        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                factorGraph, initialValues, params);
        Values result = optimizer.optimize();

        long t1_ns = System.nanoTime();
        long et_ns = t1_ns - t0_ns;

        System.out.printf("ET (ms) %f\n", (double) et_ns / 1000000);

        // for (int i = 0; i < N; ++i) {
        // Pose2 p = result.atPose2(new Key(i));
        // System.out.printf("%d %f %f %f\n", i, p.x(), p.y(), p.theta());
        // }

    }

    /**
     * As above but with C++ between factor.
     * 
     * On my machine with 10000 values this takes 1.3 sec.
     */
    @Test
    void testMultiOptimizationNative() throws Throwable {
        // repeat in order to run the profiler
        for (int j = 0; j < 5; ++j) {
            // for (int j = 0; j < 2; ++j) {
            System.gc();

            shared_ptr<Unit> noise_model = Unit.Create(3);

            // Expectation is always the same: each pose steps 1m +x.
            Pose2 expected = new Pose2(1, 0, 0);

            NonlinearFactorGraph factorGraph = new NonlinearFactorGraph();
            // prior for initial pose is origin
            factorGraph.add(PriorFactor.PriorFactorPose2(
                    new Key(0),
                    new Pose2(),
                    Diagonal.Sigmas(new Vector3(0.001, 0.001, 0.001))));

            // Create variables with bad initial values.
            // int N = 50000;
            int N = 5000;
            Values initialValues = new Values();
            for (int i = 0; i < N; ++i) {
                initialValues.insert(new Key(i), new Pose2(0, 0, 0));
                if (i > 0) {
                    factorGraph.add(BetweenFactorPose2.newBetweenFactorPose2(
                            new Key(i - 1), new Key(i), expected, noise_model));
                }
            }

            long t0_ns = System.nanoTime();

            LevenbergMarquardtParams params = new LevenbergMarquardtParams();
            LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(
                    factorGraph, initialValues, params);
            Values result = optimizer.optimize();

            long t1_ns = System.nanoTime();
            long et_ns = t1_ns - t0_ns;

            System.out.printf("ET (ms) %f\n", (double) et_ns / 1000000);

            // for (int i = 0; i < N; ++i) {
            // Pose2 p = result.atPose2(new Key(i));
            // System.out.printf("%d %f %f %f\n", i, p.x(), p.y(), p.theta());
            // }
        }
    }

}
