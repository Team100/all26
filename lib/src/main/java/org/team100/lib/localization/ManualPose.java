package org.team100.lib.localization;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverVelocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.visualization.VizUtil;
import org.wpilib.math.geometry.Pose2d;



/**
 * Provide a pose estimate with manual control.
 * 
 * This is just for testing, a way to "drive around" in simulation without using
 * a full simulated drivetrain.
 * 
 * It provides Field2d visualization of the robot using the name "robot".
 */
public class ManualPose {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private static final double MAX_V = 2.0;
    private static final double MAX_OMEGA = 2.0;
    private final DoubleArrayLogger m_log_field_robot;
    private final Supplier<DriverVelocity> m_input;
    private final ObjectCache<StateSE2> m_stateCache;
    /** Used only by update(). */
    private StateSE2 m_state;

    public ManualPose(
            LoggerFactory fieldLogger,
            Supplier<DriverVelocity> v,
            Pose2d initial) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_input = v;
        m_state = new StateSE2(initial);
        m_stateCache = Cache.of(this::update);
        LogPoller.register(this::log);
    }

    public StateSE2 getState() {
        return m_stateCache.get();
    }

    public Pose2d getPose() {
        return getState().pose();
    }

    private void log() {
        m_log_field_robot.log(this::poseArray);
    }

    private double[] poseArray() {
        Pose2d pose = getPose();
        return VizUtil.poseToArray(pose);
    }

    private StateSE2 update() {
        // Input in range [-1, 1]
        DriverVelocity input = m_input.get();
        // Velocity in m/s and rad/s
        VelocitySE2 v = VelocitySE2.scale(input, MAX_V, MAX_OMEGA);
        // Acceleration in m/s/s and rad/s/s
        AccelerationSE2 a = v.accel(m_state.velocity(), DT);
        // Find new position using *previous* velocity and accel
        // x1 = x0 + v0 dt + 1/2 a dt^2
        Pose2d pose = GeometryUtil.evolve(m_state.pose(), m_state.velocity(), a, DT);
        m_state = new StateSE2(pose, v);
        return m_state;
    }
}
