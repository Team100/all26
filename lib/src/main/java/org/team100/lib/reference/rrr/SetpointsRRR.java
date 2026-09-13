package org.team100.lib.reference.rrr;

import org.team100.lib.state.ControlRRR;

/** Current and next setpoint for RRR mechanism. */
public record SetpointsRRR(ControlRRR current, ControlRRR next) {

}
