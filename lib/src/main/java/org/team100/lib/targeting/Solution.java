package org.team100.lib.targeting;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Firing solution.
 * 
 * @param azimuth         field-relative (rad)
 * @param azimuthVelocity for feed-forward, for moving shooter or target (rad/s)
 * @param parameters      parameters for shooter
 */
public record Solution(
        Rotation2d azimuth,
        double azimuthVelocity,
        FiringParameters parameters) {
}
