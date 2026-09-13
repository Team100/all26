package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 */
public enum Experiment {
    /**
     * Smooth chassis speeds.
     */
    UseSwerveLimiter,
    /**
     * Flush network tables as often as possible. Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften,
    /**
     * Pay attention to camera input. It's useful to turn this off for testing and
     * calibration.
     */
    HeedVision,
    /**
     * Skip velocity feedforward in steering, to reduce noise.
     */
    SteerWithoutVelocity,
    /**
     * Use longitudinal dynamics, i.e. motor torque.
     */
    SwerveDynamicsLongitudinal,
    /**
     * Use lateral dynamics, i.e. slip angle.
     */
    SwerveDynamicsLateral,
    /**
     * Use only the gyro for rotation. This is useful when there's no vision input
     * to fix the gyro drift and/or odometry noise, e.g. for practice without tags.
     */
    PerfectGyro
}
