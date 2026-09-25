package org.team100.lib.visualization;

import java.util.function.Supplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Observes a pose supplier, publishes to the glass Field2d widget, which wants
 * an array of doubles, and separately to AdvantageScope, which wants a Pose2d
 * struct.
 */
public class RobotPoseVisualization {
    private final DoubleArrayLogger m_log_field_robot;
    /** For AdvantageScope, which can't understand Field2d format. */
    private final StructPublisher<Pose2d> m_pub_pose;
    private final Supplier<Pose2d> m_pose;

    public RobotPoseVisualization(
            LoggerFactory fieldLogger,
            Supplier<Pose2d> pose,
            String label) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, label);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();
        m_pose = pose;
    }

    /** Show the robot pose on AdvantageScope and Field2d. */
    public void run() {
        Pose2d pose = m_pose.get();
        double[] poseArray = VizUtil.poseToArray(pose);
        m_log_field_robot.log(() -> poseArray);
        m_pub_pose.set(pose);
    }
}
