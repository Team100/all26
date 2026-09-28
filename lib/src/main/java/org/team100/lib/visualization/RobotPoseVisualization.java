package org.team100.lib.visualization;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.StateSE2;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;



/**
 * Observes a pose supplier, publishes to the glass Field2d widget, which wants
 * an array of doubles, and separately to AdvantageScope, which wants a Pose2d
 * struct.
 */
public class RobotPoseVisualization {
    private final DoubleArrayLogger m_log_field_robot;
    /** For AdvantageScope, which can't understand Field2d format. */
    private final StructPublisher<Pose2d> m_pub_pose;
    /** Robot pose projected into the future */
    private final DoubleArrayLogger m_log_future;
    private final Supplier<StateSE2> m_state;

    public RobotPoseVisualization(
            LoggerFactory fieldLogger,
            Supplier<StateSE2> state,
            String label) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, label);
        m_log_future = fieldLogger.doubleArrayLogger(Level.TRACE, "future");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();
        m_state = state;
        LogPoller.register(this::log);
    }

    /** Show the robot pose on AdvantageScope and Field2d. */
    private void log() {
        StateSE2 state = m_state.get();
        Pose2d pose = state.pose();
        double[] poseArray = VizUtil.poseToArray(pose);
        m_log_field_robot.log(() -> poseArray);
        m_pub_pose.set(pose);
        // pose 0.1 sec in the future
        // note this would be better if it used a twist and could somehow put the
        // correct curve on the dashboard, but alas.
        Pose2d pose2 = GeometryUtil.evolve(pose, state.velocity(), AccelerationSE2.ZERO, 0.1);
        m_log_future.log(() -> VizUtil.fromPoses(List.of(pose, pose2)));

    }
}
