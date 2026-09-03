package org.team100.lib.sensor.position.absolute;

/**
 * Absolute rotational measurement, as used in, for example, swerve steering,
 * arm angles, shooter angles, etc. This does not "wind up", it only returns
 * values within [-pi, pi]. This type of sensor cannot be "reset" at runtime,
 * offsets should be fixed at instantiation.
 */
public interface RotaryPositionSensor {

    /**
     * Returns the "wrapped" angular position, i.e. this dimension is cyclical, with
     * values beyond +/- pi mapped back to the +/- pi interval: 2pi is mapped to 0,
     * 5pi/4 is mapped to pi/4, etc.
     * 
     * @return Counterclockwise-positive rad in [-pi,pi]
     */
    double getWrappedPositionRad();

    /**
     * Counts turns to derive an "unwrapped" position. Note the "zero" of this
     * measurement is arbitrary, i.e. the number of turns is zero on startup.
     * 
     * @return Counterclockwise-positive rad within an infinite domain.
     */
    double getUnwrappedPositionRad();

    /**
     * Note some velocity implementations can be noisy.
     * 
     * @return Counterclockwise positive rad/s
     */
    double getVelocityRad_S();

    /**
     * Beware of noise due to double-differentiation.
     * 
     * @return counterclockwise positive rad/s^2
     */
    double getAccelerationRad_S2();

    /**
     * For logging.
     */
    void periodic();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();
}
