package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.List;
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
import gtsam.Key;
import gtsam.NonlinearFactorGraph;
import gtsam.PlanarGyroFactor;
import gtsam.PlanarGyroFactor.PlanarGyroBiasFactor;
import gtsam.PlanarGyroFactor.PlanarGyroParams;
import gtsam.Pose2;
import gtsam.Rot2;
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
    static List<Double> odoGyro(
            SwerveDriveKinematics100 m_kinematics,
            shared_ptr<PlanarGyroParams> m_params,
            double t1,
            TimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements,
            Supplier<Rotation2d> m_gyro,
            Supplier<SwerveModulePositions> m_positions,
            NonlinearFactorGraph factors) {
        List<Double> timestamps = new ArrayList<>();
        Entry<Double, OdoGyroMeasurement> m = m_measurements.lowerEntry(t1);
        if (m == null) {
            // don't make a new state
            return timestamps;
        }
        double t0 = m.getKey();
        OdoGyroMeasurement m0 = m.getValue();

        Key x0 = Keys.poseKey(t0);
        Key x1 = Keys.poseKey(t1);
        SwerveModulePositions pos1 = m_positions.get();
        SwerveModulePositions pos0 = m0.odo();
        factors.add(getOdometryFactor(m_kinematics, x0, x1, pos0, pos1));

        // gyro factor
        Key b0 = Keys.biasKey(t0);
        Key b1 = Keys.biasKey(t1);
        Rotation2d yaw0 = m0.gyro();
        Rotation2d yaw1 = m_gyro.get();
        Rot2 dr = new Rot2(yaw1.minus(yaw0).getRadians());
        double dt = t1 - t0;
        factors.add(PlanarGyroFactor.FromRotation(x0, x1, b0, m_params, dr, dt));

        factors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                b0, b1, m_params));

        OdoGyroMeasurement m1 = new OdoGyroMeasurement(yaw1, pos1);
        m_measurements.put(t1, m1);
        timestamps.add(t1);
        return timestamps;
    }

}
