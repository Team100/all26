package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StructArrayLogger;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.trajectory.DrivetrainSplineSample;
import org.wpilib.math.trajectory.Trajectory;

public class TrajectoryVisualization {
    private final StructArrayLogger<Pose2d> m_log_trajectory;

    public TrajectoryVisualization(LoggerFactory fieldLogger) {
        m_log_trajectory = fieldLogger.structArrayLogger(Level.TRACE, "trajectory", Pose2d.struct);
    }

    public void setViz(TrajectorySE2 trajectory) {
        if (trajectory == null)
            return;
        m_log_trajectory.log(() -> trajectory.getPoints().stream().map(p -> p.point().point().waypoint().pose()).toArray(Pose2d[]::new));
    }

    public void setViz(Trajectory<DrivetrainSplineSample> m_trajectory) {
        m_log_trajectory.log(() -> m_trajectory.getSamples().stream().map(p -> p.pose).toArray(Pose2d[]::new));
    }

    public void setViz(List<Pose2d> poses) {
        m_log_trajectory.log(() -> poses.toArray(Pose2d[]::new));
    }

    public void clear() {
        m_log_trajectory.log(() -> new Pose2d[] {});
    }

}
