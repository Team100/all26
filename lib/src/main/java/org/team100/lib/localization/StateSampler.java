package org.team100.lib.localization;

import org.team100.lib.state.StateSE2;

/**
 * History sampler, to make testing easier.
 */
@FunctionalInterface
public interface StateSampler {
    /**
     * Sample the state estimate buffer.
     * 
     * @param timestampSeconds using the Takt clock
     * @return the state at that time, perhaps interpolated, may be the boundary
     *         sample if the timestamp is off either end of the buffer. Never
     *         returns null.
     */
    StateSE2 get(double timestampSeconds);
}
