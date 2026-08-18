package org.team100.lib.sensor.gyro;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.util.CanId;

/**
 * Produces real or simulated gyros depending on identity.
 */
public class GyroFactory {

    public static Gyro get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            SwerveModuleCollection collection) {
        switch (Identity.instance) {
            case SWERVE_ONE:
            case COMP_BOT:
            case BETA_BOT:
                return new ReduxGyro(parent, new CanId(60));
            default:
                // for simulation
                // this is a very high drift rate, to make it more obvious.
                // double driftRateRad_S = 0.5;
                // having proven the gyro drift works fine, the level now is
                // a bit more realistic, though still high.
                double driftRateRad_S = 0.05;
                return new SimulatedGyro(parent, kinodynamics, collection, driftRateRad_S);
        }
    }

    private GyroFactory() {
        //
    }
}
