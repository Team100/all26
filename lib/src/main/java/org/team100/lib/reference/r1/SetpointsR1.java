package org.team100.lib.reference.r1;

import org.team100.lib.state.ControlR1;

/**
 * Current and next setpoint.
 * 
 * We keep both so that feedback implementations can choose what to do. Simple
 * feedback can just compare the current setpoint to the current measurement.
 * More clever feedback might extrapolate the current measurements and compare
 * to the next setpoint.
 * 
 * @param current can be compared to current measurement
 * @param next    for future actuation
 */
public record SetpointsR1(ControlR1 current, ControlR1 next) {

}
