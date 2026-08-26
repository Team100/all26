package org.team100.lib.controller.se2;

import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;

/**
 * Feedback and feedforward control.
 */
public interface ControllerSE2 {

    /**
     * Feedback should compare the current-instant measurement to the
     * current-instant reference.
     * 
     * Feedforward should be looking at the next-step reference.
     * 
     * Previous versions of this method used a single reference for both.
     * 
     * @param measurement      Current measurement state in field coordinates
     * @param currentReference Current reference state i.e. setpoint
     * @param nextReference    Reference for dt in the future, used for feedforward.
     * @return Control output for the period during dt, so it's also what the next
     *         measurement should be. If there's no current error (i.e. feedback is
     *         zero), then this output is usually just the feedforward, i.e. the
     *         next reference velocity.
     *         Give this to SwerveDriveSubsystem.driveInFieldCoords() or something
     *         similar.
     */
    VelocityControlSE2 calculate(
            StateSE2 measurement,
            StateSE2 currentReference,
            ControlSE2 nextReference);

    /**
     * True if the error is within tolerance of the reference. The definitions of
     * "error" and "tolerance" here are implementation-specific.
     */
    boolean atReference();
}
