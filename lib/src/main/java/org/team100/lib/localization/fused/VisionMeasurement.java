package org.team100.lib.localization.fused;

import org.team100.lib.uncertainty.NoisyPose2d;

/**
 * Robot pose measurement derived from vision.
 */
public record VisionMeasurement(double timestamp, NoisyPose2d noisyMeasurement) {
}