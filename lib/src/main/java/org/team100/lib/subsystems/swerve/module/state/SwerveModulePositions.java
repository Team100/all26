package org.team100.lib.subsystems.swerve.module.state;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Container for swerve module positions.
 * 
 * This is intended to avoid passing around an array of positions,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModulePositions(
        SwerveModulePosition100 frontLeft,
        SwerveModulePosition100 frontRight,
        SwerveModulePosition100 rearLeft,
        SwerveModulePosition100 rearRight)
        implements Interpolatable<SwerveModulePositions> {

    @Override
    public SwerveModulePositions interpolate(SwerveModulePositions end, double t) {
        return new SwerveModulePositions(
                frontLeft().interpolate(end.frontLeft(), t),
                frontRight().interpolate(end.frontRight(), t),
                rearLeft().interpolate(end.rearLeft(), t),
                rearRight().interpolate(end.rearRight(), t));
    }

    /** For when you don't care about which is which. */
    public SwerveModulePosition100[] all() {
        return new SwerveModulePosition100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }

    public static SwerveModulePositions modulePositionFromDelta(
            SwerveModulePositions initial,
            SwerveModuleDeltas delta) {
        return new SwerveModulePositions(
                initial.frontLeft().plus(delta.frontLeft()),
                initial.frontRight().plus(delta.frontRight()),
                initial.rearLeft().plus(delta.rearLeft()),
                initial.rearRight().plus(delta.rearRight()));
    }

    public static SwerveModulePositions kZero() {
        return new SwerveModulePositions(
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100(),
                new SwerveModulePosition100());
    }

}
