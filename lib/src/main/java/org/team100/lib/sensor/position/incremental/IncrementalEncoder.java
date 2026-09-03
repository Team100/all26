package org.team100.lib.sensor.position.incremental;

/**
 * Represents motor-shaft encoder, probably some kind of built-in, but could
 * also represent any incremental (e.g. quadrature) encoder.
 */
public interface IncrementalEncoder {

    /**
     * "Unwrapped" angular position (rad), i.e. the measurement domain
     * continues beyond +/- pi.  May be filtered.
     * 
     * Value should be updated in Robot.robotPeriodic().
     */
    double getUnwrappedPositionRad();

    /**
     * Velocity (rad/s).
     * 
     * Note some rate implementations can be noisy.  May be filtered.
     * 
     * Value should be updated in Robot.robotPeriodic().
     */
    double getVelocityRad_S();

    /**
     * Acceleration (rad/s^2).
     * 
     * May be noisy. May be filtered.
     */
    double getAccelerationRad_S2();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the ProxyRotaryPositionSensor.
     * 
     * This is the "unwrapped" position, i.e. the domain is infinite, not cyclical
     * within +/- pi.
     * 
     * This is very slow, only use it on startup.
     * 
     * Caches should also be flushed, so the new value is available immediately.
     */
    void setUnwrappedEncoderPositionRad(double motorPositionRad);

    /** For logging */
    void periodic();

}
