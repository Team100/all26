package org.team100.lib.localization;

import java.util.Map;

import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;

import edu.wpi.first.math.geometry.Pose2d;

/** Interface for pose and velocity ("state") estimation. */
public interface StateEstimator {
    /**
     * Estimate at the given timestamp, after applying any pending updates from
     * vision or odometry.
     */
    StateSE2 getState(double timestampS);

    /**
     * Empty the pose history, add the given pose, and flush the cache.
     */
    void reset(Pose2d pose, IsotropicNoiseSE2 noise);

    /**
     * Tags outside this radius are ignored.
     */
    void setHeedRadiusM(double heedRadiusM);

    /** Timestamped estimates. */
    Map<Double, SwerveState> all();

    void close();
}
