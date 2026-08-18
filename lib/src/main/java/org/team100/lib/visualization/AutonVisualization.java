package org.team100.lib.visualization;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.trajectory.se2.TrajectorySE2;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Shows positions and trajectories.
 * 
 * TODO: make this run upon selection, not continuously.
 */
public class AutonVisualization {
    private final DoubleArrayLogger m_log_poses;
    private final DoubleArrayLogger m_log_paths;

    public AutonVisualization(LoggerFactory fieldLogger) {
        m_log_poses = fieldLogger.doubleArrayLogger(Level.COMP, "auton_poses");
        m_log_paths = fieldLogger.doubleArrayLogger(Level.COMP, "auton_paths");
    }

    public void clear() {
        m_log_poses.log(() -> new double[0]);
        m_log_paths.log(() -> new double[0]);
    }

    public void show(AnnotatedCommand cmd) {
        if (cmd == null)
            return;
        List<Pose2d> ps = new ArrayList<>();
        Pose2d p = cmd.start();
        if (p == null) {
            clear();
            return;
        }
        ps.add(p);
        List<Function<Pose2d, TrajectorySE2>> tfns = cmd.trajectoryFns();
        if (tfns.isEmpty()) {
            clear();
            return;
        }
        List<TrajectorySE2> ts = new ArrayList<>();
        for (Function<Pose2d, TrajectorySE2> tfn : tfns) {
            TrajectorySE2 t = tfn.apply(p);
            ts.add(t);
            p = t.getLastPoint().point().point().waypoint().pose();
            ps.add(p);
            // show a few samples
            for (double time = 0; time < t.duration(); time += 0.25) {
                ps.add(t.sample(time).point().point().waypoint().pose());
            }
        }
        double[] points = ts.stream()
                .map(VizUtil::fromTrajectorySE2)
                .flatMapToDouble(Arrays::stream)
                .toArray();
        m_log_paths.log(() -> points);

        double[] poses = ps.stream().map(VizUtil::poseToArray).flatMapToDouble(Arrays::stream).toArray();
        m_log_poses.log(() -> poses);
    }

}
