package org.team100.lib.subsystems.swerve.kinodynamics;

import org.team100.lib.dynamics.swerve.Tire;

/** For the practice swerve base */
public class SwerveKinodynamicsPractice extends SwerveKinodynamics {
    public SwerveKinodynamicsPractice() {
        super(
                4, // vel m/s
                10, // stall m/s/s
                2, // accel m/s/s
                2, // decel m/s/s
                0.380, // track m
                0.380, // track m
                0.445, // wheelbase m
                0.2225, // front offset m
                0.5, // vcg m
                70, // mass kg
                6, // inertia kgm^2
                new Tire(175, 0.05));
    }

}
