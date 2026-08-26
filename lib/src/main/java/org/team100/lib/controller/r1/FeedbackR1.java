package org.team100.lib.controller.r1;

import org.team100.lib.state.StateR1;

/**
 * Represents a single-output feedback controller, such as PID.
 * 
 * This could have multiple inputs, e.g. like both position and velocity, but it
 * only produces one control output, which is treated here as a primitive: you
 * can apply it wherever you want (e.g. as a control effort, or a velocity
 * target for a servo, or whatever).
 * 
 * Some of the implementations know about angle wrapping, some don't.
 */
public interface FeedbackR1 {

    double calculate(StateR1 measurement, StateR1 setpoint);

    /** True if the most-recent calculation inputs are within tolerance. */
    boolean atSetpoint();

    /** Reset controller errors, unset atSetpoint */
    void reset();

    /**
     * True if this feedback handles "wrapping" around a circle. For some uses,
     * you'd like the controller to be aware of the wrapping. For others, you do not
     * want that. In particular, OnboardAngularPositionServo handles wrapping
     * itself, and want the controller not to do so.
     */
    boolean handlesWrapping();
}
