package org.team100.lib.reference.r1;

/**
 * Velocity control only, for things like flywheels where position is
 * unimportant.
 */
public interface VelocityReferenceR1 {

    void setGoal(double goal);

    /** Set setpoint to measurement. */
    void init(double measurement);

    VelocitySetpointsR1 get();

    /** The profile has reached the goal. */
    boolean profileDone();

    /** The profile has been initialized. */
    boolean valid();
}
