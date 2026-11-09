package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.interpolation.Interpolatable;

/**
 * Verbatim measurement of wheel position and angle.
 * 
 * Verbatim measurement of yaw from the gyro, uncorrected.
 * 
 * This may be a ground-truth measurement or it may be an interpolation created
 * to match a vision timestamp.
 */
public record OdoGyroMeasurement(Rotation2d gyro, SwerveModulePositions odo)
        implements Interpolatable<OdoGyroMeasurement> {

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof OdoGyroMeasurement)) {
            return false;
        }
        OdoGyroMeasurement rec = (OdoGyroMeasurement) obj;
        return Objects.equals(odo, rec.odo)
                && Objects.equals(gyro, rec.gyro);
    }

    @Override
    public int hashCode() {
        return Objects.hash(gyro, odo);
    }

    @Override
    public String toString() {
        return "OdoGyroMeasurement [gyro=" + gyro
                + ", odo=" + odo + "]";
    }

    @Override
    public OdoGyroMeasurement interpolate(OdoGyroMeasurement endValue, double t) {
        // Check bounds on t
        if (t <= 0) {
            return this;
        }
        if (t >= 1) {
            return endValue;
        }

        // Interpolate the wheel positions.
        SwerveModulePositions startPositions = odo();
        SwerveModulePositions endPositions = endValue.odo();
        SwerveModulePositions wheelLerp = startPositions.interpolate(endPositions, t);

        // Interpolate the gyro measurement.
        Rotation2d gyroLerp = gyro().interpolate(endValue.gyro(), t);

        return new OdoGyroMeasurement(gyroLerp, wheelLerp);
    }
}