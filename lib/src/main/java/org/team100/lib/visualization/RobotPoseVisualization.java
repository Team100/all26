package org.team100.lib.visualization;

import java.util.function.Supplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Observes a pose supplier, publishes to the glass Field2d widget.
 */
public class RobotPoseVisualization implements Runnable {
    private final DoubleArrayLogger m_log_field_robot;
    private final Supplier<Pose2d> m_pose;

    public RobotPoseVisualization(
            LoggerFactory fieldLogger,
            Supplier<Pose2d> pose,
            String label) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, label);
        m_pose = pose;
    }

    @Override
    public void run() {
        m_log_field_robot.log(this::poseArray);
    }

    private double[] poseArray() {
        Pose2d pose = m_pose.get();
        return VizUtil.poseToArray(pose);
    }
}
