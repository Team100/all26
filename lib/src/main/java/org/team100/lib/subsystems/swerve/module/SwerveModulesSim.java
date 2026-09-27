package org.team100.lib.subsystems.swerve.module;

import org.team100.lib.logging.LoggerFactory;

/**
 * Uses simulated motors and simulated position sensors.
 */
public class SwerveModulesSim extends SwerveModuleCollection {
    public SwerveModulesSim(LoggerFactory log) {
        super(
                SimulatedSwerveModule100.get(log.name("Front Left")),
                SimulatedSwerveModule100.get(log.name("Front Right")),
                SimulatedSwerveModule100.get(log.name("Rear Left")),
                SimulatedSwerveModule100.get(log.name("Rear Right")));
        System.out.println("************** SIMULATED MODULES **************");
    }
}
