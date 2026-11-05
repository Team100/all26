package org.team100.lib.subsystems.swerve.kinodynamics;

import org.team100.lib.dynamics.swerve.Tire;

/**
 * For the drive base labeled `SWERVE_ONE`
 */
public class SwerveKinodynamicsSwerveOne extends SwerveKinodynamics {
    public SwerveKinodynamicsSwerveOne() {
        super(
                5, // vel m/s
                10, // stall m/s/s
                10, // max accel m/s/s
                40, // max decel m/s/s
                0.49, // front track m
                0.44, // back track m
                0.462, // wheelbase m
                0.31, // front offset m
                0.1, // vcg m
                70, // mass kg
                6, // inertia kgm^2
                new Tire(175, 0.05));
    }

}
