package org.team100.lib.localization;

import org.team100.lib.uncertainty.NoisyPose2d;

public record VisionMeasurement(double timestamp, NoisyPose2d noisyMeasurement) {
}