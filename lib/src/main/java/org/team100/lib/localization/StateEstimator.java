package org.team100.lib.localization;

import java.util.Map;

import org.team100.lib.uncertainty.NoisyStateSE2;

/** Interface for pose and velocity ("state") estimation. */
public interface StateEstimator {
    /**
     * Read pending updates from odo/gyro and vision, and apply them to the history,
     * and return an immutable copy.
     */
    ImmutableSwerveHistory update(double timestamp);

    /**
     * Estimate at the given timestamp, after applying any pending updates from
     * vision or odometry.
     */
    NoisyStateSE2 getState(double timestampS);

    /**
     * Empty the pose history, add the given pose, and flush the cache.
     * 
     * Timestamp is here for testing, it should typically be Takt.get().
     */
    void reset(double timestampS, NoisyStateSE2 state);

    /**
     * Timestamped StateSE2 estimates, for visualization.
     */
    Map<Double, NoisyStateSE2> all();

    void close();
}
