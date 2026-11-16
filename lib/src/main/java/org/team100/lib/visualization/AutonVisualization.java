package org.team100.lib.visualization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StructArrayLogger;
import org.team100.lib.trajectory.se2.TrajectorySE2;
import org.team100.lib.trajectory.se2.TrajectorySE2Entry;
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
        List<Pose2d> poseSamples = new ArrayList<>();
        Pose2d startPose = cmd.start();
        if (startPose == null) {
            clear();
            if (DEBUG)
                System.out.println("AutonVisualization: no start");
            return;
        }
        poseSamples.add(startPose);
        List<Function<Pose2d, TrajectorySE2>> tfns = cmd.trajectoryFns();
        if (tfns.isEmpty()) {
            clear();
            if (DEBUG)
                System.out.println("AutonVisualization: no trajectory fns");
            return;
        }

        List<Pose2d> pointList = new ArrayList<>();
        for (Function<Pose2d, TrajectorySE2> tfn : tfns) {
            TrajectorySE2 t = tfn.apply(startPose);
            // Use startPose here so that the next trajectory starts in the right place.
            startPose = t.getLastPoint().point().point().waypoint().pose();
            poseSamples.add(startPose);

            // Sample the path at 4 Hz.
            for (double time = 0; time < t.duration(); time += 0.25) {
                Pose2d pose = t.sample(time).point().point().waypoint().pose();
                poseSamples.add(pose);
                if (DEBUG)
                    System.out.printf("AutonVisualization: sample %f %s\n", time, pose);
            }

            // All the points along the trajectory.
            List<TrajectorySE2Entry> e = t.getPoints();
            for (TrajectorySE2Entry ee : e) {
                pointList.add(ee.point().point().waypoint().pose());
            }
        }

        if (DEBUG)
            System.out.println("AutonVisualization log path");
        m_log_paths.log(() -> pointList.toArray(Pose2d[]::new));

        if (DEBUG)
            System.out.println("AutonVisualization log samples");
        m_log_poses.log(() -> poseSamples.toArray(Pose2d[]::new));

        if (DEBUG)
            System.out.println("AutonVisualization done");
    }

}
