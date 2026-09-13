package org.team100.lib.state;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;

public record ControlRRR(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot) {
}
