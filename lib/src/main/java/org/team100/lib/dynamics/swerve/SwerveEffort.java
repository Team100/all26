package org.team100.lib.dynamics.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The swerve produces corner forces using two mechanisms:
 * 
 * * motor torque through the wheel axis to produce longitudinal force
 * * adjustment of the wheel axis to produce side force
 * 
 * @param fl front left
 * @param fr front right
 * @param rl rear left
 * @param rr rear right
 */
public record SwerveEffort(
        ModuleEffort fl, ModuleEffort fr, ModuleEffort rl, ModuleEffort rr) {
    public static SwerveEffort ZERO = new SwerveEffort(
            ModuleEffort.ZERO, ModuleEffort.ZERO,
            ModuleEffort.ZERO, ModuleEffort.ZERO);

    /**
     * Analogous to SwerveModuleState, but for force, and the adjusted angle.
     * 
     * The angle adjustment is small, so the maximpact on velocity is around
     * 0.1%, and is ignored.
     * 
     * @param f     longitudinal force, Newtons
     * @param angle corrected angle. Different from module state if there needs to
     *              be side force. Empty if there is neither velocity nor
     *              acceleration.
     */
    public record ModuleEffort(double f, Optional<Rotation2d> angle) {
        static ModuleEffort ZERO = new ModuleEffort(
                0, Optional.of(Rotation2d.kZero));

    }
}
