package org.team100.lib.subsystems.swerve.module.state;

/**
 * Container for swerve module deltas.
 * 
 * This is intended to avoid passing around an array of deltas,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModuleDeltas(
        SwerveModuleDelta frontLeft,
        SwerveModuleDelta frontRight,
        SwerveModuleDelta rearLeft,
        SwerveModuleDelta rearRight) {
    public static final SwerveModuleDeltas ZERO = new SwerveModuleDeltas(
            new SwerveModuleDelta(),
            new SwerveModuleDelta(),
            new SwerveModuleDelta(),
            new SwerveModuleDelta());

    /** For when you don't care about which is which. */
    public SwerveModuleDelta[] all() {
        return new SwerveModuleDelta[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }

    /**
     * The inverse kinematics wants this to represent a geodesic, which
     * means that the steering doesn't change between start and end.
     */
    public static SwerveModuleDeltas modulePositionDelta(
            SwerveModulePositions start,
            SwerveModulePositions end) {
        return new SwerveModuleDeltas(
                SwerveModuleDelta.delta(start.frontLeft(), end.frontLeft()),
                SwerveModuleDelta.delta(start.frontRight(), end.frontRight()),
                SwerveModuleDelta.delta(start.rearLeft(), end.rearLeft()),
                SwerveModuleDelta.delta(start.rearRight(), end.rearRight()));
    }

}
