package org.team100.lib.visualization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StructArrayLogger;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.wpilib.math.geometry.Pose2d;

/**
 * Shows positions and trajectories.
 */
public class AutonVisualization {
    private static final boolean DEBUG = false;
    private final StructArrayLogger<Pose2d> m_log_poses;
    private final StructArrayLogger<Pose2d> m_log_paths;

    public AutonVisualization(LoggerFactory fieldLogger) {
        m_log_poses = fieldLogger.structArrayLogger(Level.COMP, "auton_poses", Pose2d.struct);
        m_log_paths = fieldLogger.structArrayLogger(Level.COMP, "auton_paths", Pose2d.struct);
    }

    public void clear() {
        m_log_poses.log(() -> new Pose2d[] {});
        m_log_paths.log(() -> new Pose2d[] {});
    }

    public void show(AnnotatedCommand cmd) {
        if (cmd == null) {
            if (DEBUG)
                System.out.println("AutonVisualization: no command");
            return;
        }
        List<Pose2d> ps = new ArrayList<>();
        Pose2d p = cmd.start();
        if (p == null) {
            clear();
            if (DEBUG)
                System.out.println("AutonVisualization: no start");
            return;
        }
        ps.add(p);
        List<Function<Pose2d, TrajectorySE2>> tfns = cmd.trajectoryFns();
        if (tfns.isEmpty()) {
            clear();
            if (DEBUG)
                System.out.println("AutonVisualization: no trajectory fns");
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
                Pose2d pose = t.sample(time).point().point().waypoint().pose();
                ps.add(pose);
                if (DEBUG)
                    System.out.printf("AutonVisualization: sample %f %s\n", time, pose);
            }
        }
        Pose2d[] points = ts.stream()
                .map((m_trajectory) -> {return m_trajectory.getPoints().stream().map(p1 -> p1.point().point().waypoint().pose()).toArray(Pose2d[]::new);}).toArray(Pose2d[]::new);
        m_log_paths.log(() -> points);

        m_log_poses.log(() -> ps.toArray(Pose2d[]::new));
    }

}
