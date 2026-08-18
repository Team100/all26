package org.team100.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.dynamics.swerve.SwerveEffort;
import org.team100.lib.geometry.se2.ChassisAcceleration;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveLocalTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testSimple() throws IOException {
        Fixture fixture = new Fixture();
        SwerveLocal local = fixture.swerveLocal;
        local.setChassisSpeeds(new ChassisSpeeds(), ChassisAcceleration.ZERO);
        local.stop();
        local.setRawModuleStates(
                SwerveModuleStates.ZERO, SwerveEffort.ZERO);
        assertEquals(0, local.positions().frontLeft().distanceMeters(), DELTA);
    }
}
