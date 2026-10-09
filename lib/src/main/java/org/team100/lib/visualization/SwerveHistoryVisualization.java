package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.localization.StateEstimator;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Observes the swerve history and publishes to Field2d, in a way similar to the
 * trajectory visualizer.
 */
public class SwerveHistoryVisualization {
    private final DoubleArrayLogger m_log_history;
    private final StateEstimator m_estimator;

    public SwerveHistoryVisualization(LoggerFactory fieldLogger, StateEstimator estimator) {
        m_log_history = fieldLogger.doubleArrayLogger(Level.TRACE, "history");
        m_estimator = estimator;
        LogPoller.register(this::log);
    }

    private void log() {
        List<Pose2d> poses = m_estimator.all().values().stream().map(x -> x.state().pose()).toList();
        m_log_history.log(() -> VizUtil.fromPoses(poses));
    }
}
