package org.team100.lib.subsystems.swerve.module.state;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Container for swerve module states.
 * 
 * This is intended to avoid passing around an array of states,
 * and having to remember which location corresponds to which index.
 */
public record SwerveModuleStates(
        SwerveModuleState100 frontLeft,
        SwerveModuleState100 frontRight,
        SwerveModuleState100 rearLeft,
        SwerveModuleState100 rearRight) {
    public static final SwerveModuleStates ZERO = new SwerveModuleStates(
            SwerveModuleState100.ZERO, SwerveModuleState100.ZERO,
            SwerveModuleState100.ZERO, SwerveModuleState100.ZERO);
    private static final Optional<Rotation2d> AHEAD = Optional.of(Rotation2d.kZero);
    private static final double SLOW = 1.0;

    public static final SwerveModuleStates aheadSlow = new SwerveModuleStates(
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD),
            new SwerveModuleState100(SLOW, AHEAD));

    /** For when you don't care about which is which. */
    public SwerveModuleState100[] all() {
        return new SwerveModuleState100[] {
                frontLeft,
                frontRight,
                rearLeft,
                rearRight
        };
    }
}
