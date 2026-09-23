package org.team100.lib.experiments;

/**
 * An experiment is something that can be selectively enabled.
 */
public enum Experiment {
    /**
     * Make chassis speeds feasible.
     * 
     * Advanced drivers prefer this to be off, which allows wheel slip.
     */
    UseSwerveLimiter("Make chassis speeds feasible"),
    /**
     * Flush network tables as often as possible.
     * 
     * Do not enable this experiment in
     * competition, you'll overwhelm the network and the RIO
     */
    FlushOften("Flush network tables as often as possible"),
    /**
     * Ignore camera input.
     * 
     * Useful for testing and calibration.
     */
    IgnoreVision("Ignore camera input"),
    /**
     * Skip velocity feedforward in steering.
     * 
     * This may reduce noise.
     */
    SteerWithoutVelocity("Skip velocity feedforward in steering"),
    /**
     * Use longitudinal dynamics, i.e. motor torque.
     */
    SwerveDynamicsLongitudinal("Use longitudinal dynamics, i.e. motor torque"),
    /**
     * Use lateral dynamics, i.e. slip angle.
     */
    SwerveDynamicsLateral("Use lateral dynamics, i.e. slip angle"),
    /**
     * Use only the gyro for rotation.
     * 
     * This is useful when there's no vision input to fix the gyro drift and/or
     * odometry noise, e.g. for practice without tags.
     */
    PerfectGyro("Use only the gyro for rotation"),
    /**
     * Use simulated camera in real robot.
     * 
     * Useful for testing real-robot localization without a physical camera
     * attached. The simulated camera is used by default in simulation.
     */
    SimulateCameras("Use simulated camera in real robot"),
    /**
     * Ignore Odometry.
     * 
     * Useful for testing vision.
     */
    IgnoreOdometry("Ignore odometry input"),
    /**
     * Show seen tags.
     * 
     * Listens for camera input and paints the tags on the field.  This is
     * expensive to do, so it should be kept off for comp.
     */
    ShowTags("Show seen tags");

    /** Show this at startup */
    public final String description;

    private Experiment(String description) {
        this.description = description;
    }
}
