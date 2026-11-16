package org.team100.lib.visualization;

import java.util.function.Supplier;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.StructArrayLogger;
import org.team100.lib.logging.LoggerFactory.StructLogger;
import org.team100.lib.state.StateSE2;
import org.wpilib.math.geometry.Pose2d;

/**
 * Observes a pose supplier, publishes to the glass Field2d widget, which wants
 * an array of doubles, and separately to AdvantageScope, which wants a Pose2d
 * struct.
 */
public class RobotPoseVisualization {
    /**
     * Robot pose.
     * 
     * The Field2d example always uses arrays, even for a single pose, but a single
     * struct works too.
     */
    private final StructLogger<Pose2d> m_log_field_robot;

    /** Robot pose projected into the future. */
    private final StructArrayLogger<Pose2d> m_log_future;

    /** Supplier of robot state, e.g. from the pose estimator. */
    private final Supplier<StateSE2> m_state;

    public RobotPoseVisualization(
            LoggerFactory fieldLogger,
            Supplier<StateSE2> state,
            String label) {
        m_log_field_robot = fieldLogger.structLogger(Level.COMP, label, Pose2d.struct);
        m_log_future = fieldLogger.structArrayLogger(Level.COMP, label + "_future", Pose2d.struct);
        m_state = state;
        LogPoller.register(this::log);
    }

    /** Show the robot pose on AdvantageScope and Field2d. */
    private void log() {
        StateSE2 state = m_state.get();
        Pose2d pose = state.pose();
        m_log_field_robot.log(() -> pose);
        // pose 0.1 sec in the future
        // note this would be better if it used a twist and could somehow put the
        // correct curve on the dashboard, but alas.
        Pose2d pose2 = GeometryUtil.evolve(pose, state.velocity(), AccelerationSE2.ZERO, 0.1);
        m_log_future.log(() -> new Pose2d[] { pose, pose2 });
    }
}
