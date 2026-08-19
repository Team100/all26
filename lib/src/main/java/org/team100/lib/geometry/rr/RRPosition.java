package org.team100.lib.geometry.rr;

import org.wpilib.math.geometry.Translation2d;

/**
 * Cartesian position of each joint.
 */
public record RRPosition(Translation2d p1, Translation2d p2) {
}