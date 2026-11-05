package org.team100.lib.subsystems.swerve.kinodynamics;

import org.team100.lib.dynamics.swerve.Tire;

/**
 * For the Calgames Team 9999 comp bot .
 * 
 * these numbers are a guess based on the betabot numbers.
 * the comp bot uses the "fast" ratio and FOC falcons
 * so should be a bit higher top speed and less acceleration.
 * note these measurements were updated jun 24.
 * 9/24/24, raised steering rate from 20 to 40, accel from 60 to 120.
 * 3/15/26, lowered vcg, fixed offset, rasied other limits
 */
public class SwerveKinodynamicsComp9999 extends SwerveKinodynamics {
    public SwerveKinodynamicsComp9999() {
        super(
                5, // max vel m/s
                20, // stall m/s/s
                20, // max accel m/s/s
                50, // max decel m/s/s
                0.565, // front track m
                0.565, // back track m
                0.565, // wheelbase m
                0.283, // front offset m
                0.15, // vcg m
                70, // mass kg
                6, // inertia kgm^2
                new Tire(175, 0.05));
    }

}
