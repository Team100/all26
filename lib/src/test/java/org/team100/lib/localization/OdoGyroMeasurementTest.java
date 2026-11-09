package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Rotation2d;

public class OdoGyroMeasurementTest {

    @Test
    void testInterp0() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()));
        Rotation2d gyroYaw0 = new Rotation2d();
        OdoGyroMeasurement measurement0 = new OdoGyroMeasurement(gyroYaw0, p0);
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);
        OdoGyroMeasurement measurement1 = new OdoGyroMeasurement(gyroYaw1, p1);
        {
            // t=0 should return r0. note "t" is not time
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 0.0);
            assertEquals(0, lerp.gyro().getRadians(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 1.0);
            assertEquals(1, lerp.gyro().getRadians(), 0.001);
        }
        {
            // t=0.5 is just halfway there.
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 0.5);
            assertEquals(0.5, lerp.gyro().getRadians(), 0.001);
        }
    }

    @Test
    void testInterp1() {
        // initally driving straight
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())));
        Rotation2d gyroYaw0 = new Rotation2d();
        OdoGyroMeasurement measurement0 = new OdoGyroMeasurement(gyroYaw0, p0);
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);

        OdoGyroMeasurement measurement1 = new OdoGyroMeasurement(gyroYaw1, p1);
        {
            // t=0 should return r0. note "t" is not time
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 0.0);
            assertEquals(0, lerp.gyro().getRadians(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 1.0);
            assertEquals(1, lerp.gyro().getRadians(), 0.001);
        }
        {
            // t=0.5
            OdoGyroMeasurement lerp = measurement0.interpolate(measurement1, 0.5);
            assertEquals(0.5, lerp.gyro().getRadians(), 0.001);
        }
    }

}
