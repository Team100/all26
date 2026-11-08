package org.team100.lib.localization.simulation;

import java.util.function.Supplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StateSE2Logger;
import org.team100.lib.state.StateSE2;

/**
 * Logs the difference between two states.
 * 
 * The error is calculated as "estimate minus reference" which is the opposite
 * of the usual control-theory sign for error, because here we're interested in
 * evaluating the quality of the estimate.
 * 
 * TODO: add summary metrics, see Metrics.java
 */
public class StateError {
    /** Ground-truth reference */
    private final Supplier<StateSE2> m_ref;
    /** Pose-estimator output */
    private final Supplier<StateSE2> m_est;
    private final StateSE2Logger m_log;

    public StateError(
            LoggerFactory logger,
            Supplier<StateSE2> ref,
            Supplier<StateSE2> est) {
        m_ref = ref;
        m_est = est;
        m_log = logger.type(this).StateSE2Logger(Level.TRACE, "error");
        LogPoller.register(this::log);
    }

    private void log() {
        m_log.log(() -> m_est.get().minus(m_ref.get()));
    }

}
