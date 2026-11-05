package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.SwerveModuleCollection;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveLocalTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testSimple() throws IOException {
        LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        // uses simulated modules
        SwerveModuleCollection collection = SwerveModuleCollection.get(
                logger, currentLog, new CurrentLimit(10, 20), new CurrentLimit(10, 20));
        SwerveLocal swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        swerveLocal.setChassisSpeeds(new ChassisSpeeds(), ChassisAcceleration.ZERO);
        swerveLocal.stop();
        swerveLocal.setRawModuleStates(
                SwerveModuleStates.ZERO, SwerveEffort.ZERO);
        assertEquals(0, swerveLocal.positions().frontLeft().distanceMeters(), DELTA);
    }
}
