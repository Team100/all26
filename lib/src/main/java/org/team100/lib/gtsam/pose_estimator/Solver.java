package org.team100.lib.gtsam.pose_estimator;

import gtsam.BatchFixedLagSmoother;
import gtsam.FixedLagSmoother;
import gtsam.Key;
import gtsam.KeyVector;
import gtsam.Marginals;
import gtsam.Matrix;
import gtsam.NonlinearFactor;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector;
import gtsam.Vector3;
import gtsam.shared_ptr;

/**
 * Port of estimate.py from 2024.
 */
public class Solver {
    private static final boolean DEBUG = false;
    private final BatchFixedLagSmoother m_smoother;
    public final NonlinearFactorGraph m_newFactors;
    public final Values m_newValues;
    /** key is Key, "X(timestamp in us)", value is timestamp in us */
    public final FixedLagSmoother.KeyTimestampMap m_newTimestamps;

    private Values m_result;

    /** @param lag in microseconds, not seconds as in python */
    public Solver(double lag) throws Throwable {
        // Initialize the model
        // initial module positions are at their origins.
        // TODO: some other initial positions?

        m_smoother = new BatchFixedLagSmoother(lag);
        m_result = new Values();
        // between updates we accumulate inputs here

        m_newFactors = new NonlinearFactorGraph();
        m_newValues = new Values();
        m_newTimestamps = new FixedLagSmoother.KeyTimestampMap();
    }

    public void addVariable(Key key, double time_us, Pose2 initial_value) throws Throwable {
        if (DEBUG) {
            System.out.print("adding key:\n");
            key.print();
            System.out.printf("with value (%f %f %f)\n",
                    initial_value.x(), initial_value.y(), initial_value.theta());
        }
        if (exists(key))
            return;
        m_newValues.insert(key, initial_value);
        m_newTimestamps.put(key, time_us);
        if (DEBUG)
            System.out.println("added!");
    }

    public void addVariable(Key key, double time_us, double initial_value) throws Throwable {
        if (DEBUG) {
            System.out.print("adding key:\n");
            key.print();
            System.out.printf("with value: %f\n", initial_value);
        }
        if (exists(key))
            return;
        m_newValues.insert(key, initial_value);
        if (DEBUG)
            System.out.printf("adding timestamp %f\n", time_us);
        m_newTimestamps.put(key, time_us);
        if (DEBUG)
            System.out.println("added!");
    }

    private boolean exists(Key k) throws Throwable {
        return m_result.exists(k) || m_newValues.exists(k);
    }

    /**
     * Add a factor to the graph.
     * The variables referenced must exist.
     * returns true if successful.
     */
    public <T extends NonlinearFactor> boolean add(shared_ptr<T> f)
            throws Throwable {
        KeyVector keys = f.get().keys();
        for (int i = 0; i < keys.size(); ++i) {
            Key k = keys.at(i);
            if (!m_newValues.exists(k) && !m_result.exists(k)) {
                complain(f, k);
                return false;
            }
        }
        if (DEBUG) {
            System.out.println("adding factor");
            f.get().print();
        }
        m_newFactors.add(f);
        return true;
    }

    private <T extends NonlinearFactor> void complain(
            shared_ptr<T> f, Key k) throws Throwable {
        System.out.println("FATAL ERROR!");
        System.out.println("factor:");
        System.out.flush();
        f.get().print();
        System.out.flush();
        System.out.println("key does not exist in results or new values:");
        System.out.flush();
        k.print();
        System.out.println("result:");
        m_result.print("result");
        System.out.println("new values:");
        m_newValues.print("new values");
    }

    /**
     * Run the solver
     */
    public void update() throws Throwable {
        if (DEBUG) {
            System.out.println("update");
            m_newFactors.print("new factors");
            m_newValues.print("new values");
        }
        m_smoother.update(m_newFactors, m_newValues, m_newTimestamps);
        m_result = m_smoother.calculateEstimate();

        // reset the accumulators
        m_newFactors.resize(0);
        m_newValues.clear();
        m_newTimestamps.clear();
    }

    public long result_size() throws Throwable {
        // result.print();
        return m_result.size();
    }

    public Values result() {
        return m_result;
    }

    /** The mean expected pose. */
    public Pose2 mean_pose2(Key key) throws Throwable {
        return m_result.atPose2(key);
    }

    public double mean_double(Key key) throws Throwable {
        if (DEBUG) {
            System.out.println("Looking for double key:");
            key.print();
            System.out.println("In results:");
            m_result.print("");
        }
        return m_result.atDouble(key);
    }

    public Vector sigma_pose2(Key key) throws Throwable {
        Marginals m = marginal_covariance();
        Matrix s = m.marginalCovariance(key);
        return s.diagonal_cwiseSqrt();
    }

    public Marginals marginal_covariance() throws Throwable {
        NonlinearFactorGraph factors = getFactors();
        // QR seems stable, default (Cholesky) is not.
        return Marginals.QR(factors, m_result);
    }

    public NonlinearFactorGraph getFactors() throws Throwable {
        return m_smoother.getFactors();
    }

    public Pose2 sample_Pose2(Key key) throws Throwable {
        Marginals marginals = marginal_covariance();
        Matrix cov = marginals.marginalCovariance(key);
        Vector3 t = new Vector3(cov.draw());
        return mean_pose2(key).expmap(t);
    }

}
