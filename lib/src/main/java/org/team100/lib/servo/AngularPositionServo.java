package org.team100.lib.servo;

import org.team100.lib.music.Player;
import org.team100.lib.state.StateR1;

/**
 * Angular position control, e.g. for swerve steering axes or arm axes.
 * 
 * The "servo" layer wraps the mechanism control with a profile, if desired.
 * 
 * An angular servo should generally get "wrapped" input. It figures out what
 * "unwrapped" commands to give the underlying mechanism.
 */
public interface AngularPositionServo extends Player {
    /**
     * Zeros controller errors.
     * Sets reference setpoint and goal to current measurement.
     * Unsets the servo setpoint.
     *
     * It is ESSENTIAL TO CALL RESET in your INITIALIZE logic, so that the reference
     * starts with the current measurement. Otherwise, the reference will "remember"
     * whatever it was doing before.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * Invalidates the current profile.
     * 
     * You need to keep calling this to keep actuating.
     */
    void setDutyCycle(double dutyCycle);

    /**
     * Limit torque via the motor current limit.
     */
    void setTorqueLimit(double torqueNm);

    /**
     * Initializes the profile if necessary.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * uses the current and next setpoints for actuation.
     * 
     * Because it uses a profile, this method is not suitable for anything that
     * needs to be coordinated, e.g. with another mechanism, or with a target. It's
     * suitable for simple, smooth control of independent mechanisms.
     * 
     * Gravity/spring compensation used to be here; it should be in the
     * dynamics now.
     * 
     * @param wrappedGoalRad The goal angle here wraps within [-pi, pi], using
     *                       output measurements, e.g. shaft radians, not motor
     *                       radians.
     */
    void setPositionProfiled(double wrappedGoalRad);

    /**
     * Invalidates the current profile, sets the setpoint directly, using the
     * supplied position and zero for acceleration.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * This is really only appropriate for the Outboard control case, because the
     * feedback controller there can be much firmer than in the Outboard case, but
     * it does work with Onboard feedback.
     * 
     * This method goes the "short way" directly and the "long way" with a profile.
     * 
     * Gravity/spring compensation used to be here; it should be in the
     * dynamics now.
     * 
     * @param wrappedGoalRad Where the mechanism should be at the next time step.
     *                       The angle here wraps within [-pi, pi], using
     *                       output measurements, e.g. shaft radians, not motor
     *                       radians.
     * @param velocityRad_S  The desired velocity at the next time step, used for
     *                       feedforward.
     */
    void setPositionDirect(double wrappedGoalRad, double velocityRad_S);

    /**
     * For unwrapped goal.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * Gravity/spring compensation used to be here; it should be in the
     * dynamics now.
     */
    void actuateWithProfile(double unwrappedGoalX);

    /**
     * Unwrapped setpoint, no profile.
     * 
     * You need to keep calling this to keep actuating.
     * 
     * Gravity/spring compensation used to be here; it should be in the
     * dynamics now.
     */
    void actuateDirect(double unwrappedSetpoint);

    /**
     * This is the "wrapped" value, i.e. it is periodic within +/- pi.
     * 
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return Current position measurement, radians.
     */
    double getWrappedPositionRad();

    /** The "unwrapped" value domain is infinite. */
    double getUnwrappedPositionRad();

    /** For testing. */
    StateR1 getUnwrappedGoal();

    /** A valid setpoint exists. Not true when the goal is inaccessible. */
    boolean validSetpoint();

    /** Mechanism is following the desired setpoint. */
    boolean atSetpoint();

    boolean profileDone();

    /**
     * Profile is done, and we're on the setpoint.
     * 
     * Note this is affected by the setpoint update.
     * 
     * Should be false after reset.
     * 
     * It really makes the most sense to call this *before* updating the setpoint,
     * because the measurement comes from the recent-past Takt and the updated
     * setpoint will be aiming at the next one.
     */
    boolean atGoal();

    void stop();

    void close();

    /** for logging */
    void periodic();

    /** For setting friction only */
    void setVelocity(double rad_S);
}
