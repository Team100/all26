package org.team100.lib.reference.se2;

import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;

/**
 * A source of references for control in SE2, useful for drivetrain or
 * planar mechanism.
 */
public interface ReferenceSE2 {

    /**
     * Make current() return the starting point, which might be informed by the
     * supplied measurement, or not.
     */
    void initialize(StateSE2 measurement);

    /**
     * Reference for the current time. This can be null (e.g. between instantiation
     * and initialization).
     */
    StateSE2 current();

    /**
     * Reference for 0.02 sec in the future. This can be null (e.g. between
     * instantiation and initialization).
     */
    ControlSE2 next();

    /**
     * The reference series has reached the end, and will keep returning the same
     * (goal) reference.
     */
    boolean done();

    /**
     * For reference sources that have endpoints, what is the endpoint?
     */
    StateSE2 goal();
}
