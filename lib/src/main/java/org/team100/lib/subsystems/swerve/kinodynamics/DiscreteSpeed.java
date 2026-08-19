package org.team100.lib.subsystems.swerve.kinodynamics;

import org.wpilib.math.geometry.Twist2d;

/**
 * Robot-relative speed to apply (constantly) for dt to achieve the
 * instantaneous speed desired.
 */
public record DiscreteSpeed(Twist2d twist, double dt) {
}
