package org.team100.lib.servo;

/**
 * Linear position control, e.g. for elevators.
 * 
 * The "servo" layer wraps the mechanism control with a profile, if desired.
 */
public interface LinearPositionServo {
    /**
     * Zeros controller errors, sets setpoint and goal to current measurement.
     * 
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /** Force the encoder measurement. For "homing". */
    void setEncoderPositionM(double positionM);

    /** For measuring friction. */
    void setVoltage(double volts);

    /** For friction measurement and homing. */
    void setVelocity(double velocityM_S);

    /**
     * Initializes the profile if necessary.
     * This is movement and force on the output.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * Gravity compensation used to be here; it should be in the
     * dynamics now.
     * 
     * @param goalM meters
     */
    void setPositionProfiled(double goalM);

    /**
     * Invalidates the current profile, sets the setpoint directly.
     * This takes both current and next setpoints so that the implementation can
     * choose the current one for feedback and the next one for feedforward.
     *
     * You need to keep calling this to keep actuating.
     * 
     * Gravity compensation used to be here; it should be in the
     * dynamics now.
     */
    void setPositionDirect(double goalM);

    double getPosition();

    double getVelocity();

    double getAcceleration();

    boolean atSetpoint();

    /** Useful for sequencing, without waiting for the controller. */
    boolean profileDone();

    /** Profile is done and we're on the setpoint. */
    boolean atGoal();

    void stop();

    void close();
}
