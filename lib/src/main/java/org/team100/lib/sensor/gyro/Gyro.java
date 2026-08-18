package org.team100.lib.sensor.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Three-axis gyro, NWU.
 */
public interface Gyro {

    /**
     * White noise standard deviation of the gyro rate measurement, in rad/s.
     *
     * Implementations should do their own scaling according to whatever sample
     * averaging they do of the rate itself.
     * 
     * To find the standard deviation in a delta between yaw measurements, multiply
     * this value by the delta period.
     * 
     * Typical published values range from 1e-4 to 5e-3 rad/s.
     * 
     * @see https://stechschulte.net/2023/10/11/imu-specs.html
     */
    double white_noise();

    /**
     * Noise standard deviation of the gyro bias (i.e. drift), in rad/s.
     * 
     * This is the "bias stability", typically 10 degrees/hour or 5e-5 rad/s.
     * 
     * @see https://stechschulte.net/2023/10/11/imu-specs.html
     */
    double bias_noise();

    /**
     * Yaw in radians, NWU, counterclockwise positive.
     * Implementations should extrapolate using the yaw rate,
     * to get the yaw at the current Takt time.
     */
    Rotation2d getYawNWU();

    /**
     * Yaw rate in rad/s, NWU, counterclockwise positive.
     * Not cached, may be inconsistent with the yaw value, and not constant during
     * the cycle.
     */
    double getYawRateNWU();

    /** Pitch in radians, NWU, positive-down. */
    Rotation2d getPitchNWU();

    /** Roll in radians, NWU, positive-right. */
    Rotation2d getRollNWU();

    /** For computing rate. */
    void periodic();

}
