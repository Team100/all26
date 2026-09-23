package org.team100.lib.localization;

import org.team100.lib.uncertainty.NoisyPose2d;

/** For testing. */
interface VisionUpdater {

    /**
     * Put a new state estimate based on the supplied measurement. If not current,
     * subsequent wheel updates are replayed.
     */
    void put(
            double timestampS,
            NoisyPose2d noisyMeasurement);

}