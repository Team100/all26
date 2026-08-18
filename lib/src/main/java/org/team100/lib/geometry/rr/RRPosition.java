package org.team100.lib.geometry.rr;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Cartesian position of each joint.
 */
public record RRPosition(Translation2d p1, Translation2d p2) {
}