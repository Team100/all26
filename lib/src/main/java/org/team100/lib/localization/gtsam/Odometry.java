package org.team100.lib.localization.gtsam;

import java.util.Map.Entry;
import java.util.function.Supplier;

import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.util.TimeInterpolatableBuffer100;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.BetweenFactorPose2;
import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Pose2;
import gtsam.Rot2;
import gtsam.Values;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

public class Odometry {

    static shared_ptr<BetweenFactorPose2> getOdometryFactor(
            SwerveDriveKinematics100 m_kinematics,
            Key x0,
            Key x1,
            SwerveModulePositions m0,
            SwerveModulePositions m1) {
        SwerveModuleDeltas deltas = SwerveModuleDeltas.modulePositionDelta(m0, m1);
        Twist2d twist = m_kinematics.forward(deltas);
        Vector3 twistVector = new Vector3(twist.dx, twist.dy, twist.dtheta);
        Pose2 measurement = new Pose2().expmap(twistVector);
        return BetweenFactorPose2.newBetweenFactorPose2(
                x0, x1, measurement, noise(twistVector));
    }

    /** Speed-dependent noise. */
    static shared_ptr<Diagonal> noise(Vector3 twist) {
        double distance = twist.norm();
        return Diagonal.Sigmas(
                new Vector3(
                        noise(distance),
                        noise(distance),
                        noise(distance)));
    }

    /** Speed-dependent noise. */
    private static double noise(double distance) {
        // this used to be 0.0001 but gtsam didn't like it
        return 0.001 + 0.02 * distance;
    }

    /**
     * Add factor for odo/gyro measurement
     * 
     * TODO: move the constants here to the class, make this not static.
     */
    static void odoGyro(
            SwerveDriveKinematics100 m_kinematics,
            shared_ptr<PlanarGyroParams> m_params,
            double timestamp,
            TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements,
            Supplier<Rotation2d> m_gyro,
            Supplier<SwerveModulePositions> m_positions,
            NonlinearFactorGraph m_newFactors,
            Values m_newValues,
            KeyTimestampMap m_newTimestamps) {
        Entry<Double, OdoGyroMeasurement> measurementEntry = m_measurements.lowerEntry(timestamp);
        if (measurementEntry == null) {
            return;
        }
        double previousTimestamp = measurementEntry.getKey();
        OdoGyroMeasurement previousMeasurement = measurementEntry.getValue();

        double dt = timestamp - previousTimestamp;

        Key x0 = Keys.poseKey(previousTimestamp);
        Key x1 = Keys.poseKey(timestamp);

        Key b0 = Keys.biasKey(previousTimestamp);
        Key b1 = Keys.biasKey(timestamp);

        OdoGyroMeasurement newMeasurement = new OdoGyroMeasurement(m_gyro.get(), m_positions.get());

        m_newFactors.add(getOdometryFactor(m_kinematics,
                x0, x1, previousMeasurement.odo(), newMeasurement.odo()));

        Rot2 dr = new Rot2(newMeasurement.gyro().minus(
                previousMeasurement.gyro()).getRadians());
        m_newFactors.add(PlanarGyroFactor.FromRotation(
                x0, x1, b0, m_params, dr, dt));

        m_newFactors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                b0, b1, m_params));

        m_measurements.put(timestamp, newMeasurement);
    }

}
