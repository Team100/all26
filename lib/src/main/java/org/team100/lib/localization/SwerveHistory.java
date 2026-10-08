package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.SortedMap;

import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * History is just a container, in fact the implementation is little more than a
 * wrapper around TimeInterpolatableBuffer.
 * 
 * The history always has *something* in it, even the initial zero pose.
 * 
 * The buffer only needs to be long enough to catch stale-but-still-helpful
 * vision updates.
 * 
 * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
 * code with about 75-100 ms latency. There will never be a vision update
 * older than about 200 ms.
 */
public class SwerveHistory implements StateSampler {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_timestamp;
    private final TimeInterpolatableBuffer100<SwerveState> m_poseBuffer;

    public SwerveHistory(
            LoggerFactory parent,
            double bufferDuration,
            Rotation2d gyroAngle,
            VariableR1 gyroBias,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            IsotropicNoiseSE2 noise,
            double timestampSeconds) {
        m_log_timestamp = parent.type(this).doubleLogger(Level.TRACE, "sample timestamp");
        SwerveStateInterpolator interpolator = new SwerveStateInterpolator();
        StateSE2 state = new StateSE2(initialPoseMeters, VelocitySE2.ZERO);
        SwerveState initialState = new SwerveState(
                state, noise, modulePositions, gyroAngle, gyroBias, null);
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                interpolator, bufferDuration, timestampSeconds, initialState);
    }

    public SwerveState getExact(double t) {
        return m_poseBuffer.getExact(t);
    }

    /**
     * Sample the state estimate buffer. May return an entry exactly, if the
     * timestamp matches a known timestamp exactly. Otherwise returns an
     * interpolated value.
     */
    @Override
    public StateSE2 get(double timestampSeconds) {
        m_log_timestamp.log(() -> timestampSeconds);
        return m_poseBuffer.get(timestampSeconds).state();
    }

    /** Empty the buffer and add the given measurements. */
    void reset(
            SwerveModulePositions modulePositions,
            Pose2d pose,
            IsotropicNoiseSE2 noise,
            double timestampSeconds,
            Rotation2d gyroYaw,
            VariableR1 gyroBias) {
        StateSE2 newState = new StateSE2(pose, VelocitySE2.ZERO);
        SwerveState state = new SwerveState(
                newState,
                noise,
                modulePositions,
                gyroYaw,
                gyroBias,
                null);
        m_poseBuffer.reset(timestampSeconds, state);
    }

    //////////////////////////////////////////////////
    //
    // Methods below are for history maintenance and testing.

    /**
     * @param timestamp time, seconds
     * @param state     from odometry or vision
     */
    void put(double timestamp, SwerveState state) {
        if (DEBUG)
            System.out.printf("SwerveHistory.put() %f %s\n", timestamp, state);
        m_poseBuffer.put(timestamp, state);
    }

    /** Entry for time strictly before timestamp. Never interpolated. */
    Entry<Double, SwerveState> lowerEntry(double timestamp) {
        return m_poseBuffer.lowerEntry(timestamp);
    }

    /**
     * Sample the buffer at the given time. May return an entry exactly, if the
     * timestamp matches a known timestamp exactly. Otherwise returns an
     * interpolated value. Don't use this method outside this package.
     */
    SwerveState getRecord(double timestamp) {
        return m_poseBuffer.get(timestamp);
    }

    boolean tooOld(double timestamp) {
        return m_poseBuffer.tooOld(timestamp);
    }

    /** SwerveStates strictly later than the timestamp. */
    public SortedMap<Double, SwerveState> exclusiveTailMap(double timestamp) {
        return m_poseBuffer.tailMap(timestamp, false);
    }

    int size() {
        return m_poseBuffer.size();
    }

    double lastKey() {
        return m_poseBuffer.lastKey();
    }

    /** Print the buffer contents. */
    void dump() {
        m_poseBuffer.dump();
    }

}
