package org.team100.lib.servo;

import org.team100.lib.music.Player;

/**
 * Linear velocity control, e.g. for drive wheels, flywheels, etc.
 * 
 * The "servo" layer wraps the mechanism control with a profile, if desired.
 * 
 * Represents a servo whose output is measured in linear units -- this is
 * usually relevant for wheeled mechanisms, where the surface speed of the wheel
 * is the important thing. Examples:
 * 
 * A conveyor belt drive is a wheel, but the important thing is the linear
 * movement of the belt.
 * 
 * A ball-shooter is a wheel, but the important thing is its surface speed.
 */
public interface LinearVelocityServo extends Player {
    /** Reset encoder to zero */
    void reset();

    void setDutyCycle(double dutyCycle);

    /**
     * Use a velocity profile.
     * 
     * You need to keep calling this to keep actuating.
     */
    void setVelocityProfiled(double goalM_S);

    /**
     * Sets velocity without a profile.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * @param setpointM_S  desired speed, m/s
     * @param setpointM_S2 desired acceleration m/s^2
     */
    void setVelocityDirect(double setpointM_S, double setpointM_S2);

    /** meters/sec. Note this can be noisy, maybe filter it. */
    double getVelocity();

    /** Near the profile setpoint */
    boolean atSetpoint();

    /** Useful for sequencing, without waiting for the controller. */
    boolean profileDone();

    /** Within some tolerance of the desired */
    boolean atGoal();

    void stop();

    /** Meters. Implementations should use the Cache mechanism. */
    double getDistance();

    /** For cleanup. */
    void close();

    /** For logging */
    void periodic();

}
