package org.team100.lib.localization.simulation;

import java.util.function.Supplier;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.state.StateSE2;

/**
 * Logs the difference between two states.
 * 
 * The error is calculated as "estimate minus reference" which is the opposite
 * of the usual control-theory sign for error, because here we're interested in
 * evaluating the quality of the estimate.
 */
public class StateError {
    /** Ground-truth reference */
    private final Supplier<StateSE2> m_ref;
    /** Pose-estimator output */
    private final Supplier<StateSE2> m_est;
    private final StateSE2Logger m_log_stateError;
    private final DoubleLogger m_log_poseDistanceError;
    private final DoubleLogger m_log_speedError;

    public StateError(
            LoggerFactory parent,
            Supplier<StateSE2> ref,
            Supplier<StateSE2> est) {
        m_ref = ref;
        m_est = est;
        LoggerFactory log = parent.type(this);
        m_log_stateError = log.StateSE2Logger(Level.TRACE, "state");
        m_log_poseDistanceError = log.doubleLogger(Level.TRACE, "distance");
        m_log_speedError = log.doubleLogger(Level.TRACE, "speed");
        LogPoller.register(this::log);
    }

    private void log() {
        m_log_stateError.log(
                () -> m_est.get().minus(m_ref.get()));
        m_log_poseDistanceError.log(
                () -> Metrics.translationalDistance(
                        m_est.get().pose(), m_ref.get().pose()));
        m_log_speedError.log(
                () -> m_est.get().velocity().minus(m_ref.get().velocity()).norm());
    }

}
