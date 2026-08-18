package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.trajectory.se2.TrajectorySE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TrajectoryVisualization {
    private final DoubleArrayLogger m_log_trajectory;

    public TrajectoryVisualization(LoggerFactory fieldLogger) {
        m_log_trajectory = fieldLogger.doubleArrayLogger(Level.TRACE, "trajectory");
    }

    public void setViz(TrajectorySE2 trajectory) {
        if (trajectory == null)
            return;
        m_log_trajectory.log(() -> VizUtil.fromTrajectorySE2(trajectory));
    }

    public void setViz(Trajectory m_trajectory) {
        m_log_trajectory.log(() -> VizUtil.fromWPITrajectory(m_trajectory));
    }

    public void setViz(List<Pose2d> poses) {
        m_log_trajectory.log(() -> VizUtil.fromPoses(poses));
    }

    public void clear() {
        m_log_trajectory.log(() -> new double[0]);
    }

}
