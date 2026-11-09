package org.team100.lib.localization.gtsam;

import java.util.Map.Entry;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.localization.OdoGyroMeasurement;
import org.team100.lib.localization.simulation.AddOdometryNoise;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.util.DiscreteTimeInterpolatableBuffer100;
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
    private final SwerveDriveKinematics100 m_kinematics;

    /**
     * Noise source for simulation.
     * For a real robot, use UnaryOperator.identity().
     */
    private final UnaryOperator<Twist2d> m_noise;
    private final Supplier<Rotation2d> m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;

    public Odometry(SwerveDriveKinematics100 kinematics, boolean noisy,
            Supplier<Rotation2d> gyro,
            Supplier<SwerveModulePositions> positions) {
        m_kinematics = kinematics;
        m_noise = noisy ? new AddOdometryNoise() : UnaryOperator.identity();
        m_gyro = gyro;
        m_positions = positions;
    }

    shared_ptr<BetweenFactorPose2> getOdometryFactor(
            SwerveDriveKinematics100 m_kinematics,
            Key x0,
            Key x1,
            SwerveModulePositions m0,
            SwerveModulePositions m1) {
        SwerveModuleDeltas deltas = SwerveModuleDeltas.modulePositionDelta(m0, m1);
        Twist2d twist = m_kinematics.forward(deltas);
        // Add noise if in sim
        twist = m_noise.apply(twist);
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
        if (Experiments.INSTANCE.enabled(Experiment.IgnoreOdometry)) {
            // very high noise is like ignoring the signal.
            return 1;
        }
        // this used to be 0.0001 but gtsam didn't like it
        return 0.001 + 0.02 * distance;
    }

    /**
     * Add factor, new values, and timestamps, for odo/gyro measurement.
     * 
     * This always creates a measurement, new timestamps, and new values.
     * 
     * If possible, it creates a factor linking the new measurement with
     * the previous one.
     * 
     * TODO: move the constants here to the class, make this not static.
     */
    void odoGyro(
            Values m_result,
            shared_ptr<PlanarGyroParams> m_params,
            double t1,
            DiscreteTimeInterpolatableBuffer100<OdoGyroMeasurement> m_measurements,
            NonlinearFactorGraph factors,
            Values newValues,
            KeyTimestampMap newTimestamps) {

        long t1us = (long) (t1 * 1e6);

        // Record the new measurement.
        Rotation2d yaw1 = m_gyro.get();
        SwerveModulePositions pos1 = m_positions.get();
        OdoGyroMeasurement m1 = new OdoGyroMeasurement(yaw1, pos1);
        m_measurements.put(t1us, m1);

        // Keys for the new time.
        Key x1 = Keys.poseKey(t1us);
        Key b1 = Keys.biasKey(t1us);

        // Record timestamps of the new measurement.
        newTimestamps.put(x1, t1);
        newTimestamps.put(b1, t1);

        // Add values for the new timestamp.
        Entry<Long, OdoGyroMeasurement> m = m_measurements.lowerEntry(t1us);
        if (m == null) {
            // There is no previous state.
            // This should only happen for the initial update.
            //
            // Use zeroes as the initial values.
            Pose2 pose1 = new Pose2();
            newValues.insert(x1, pose1);
            Priors.posePrior(factors, x1, pose1);
            double bias1 = 0.0;
            newValues.insert(b1, bias1);
            Priors.biasPrior(factors, b1, bias1);
            return;
        }

        // We found a previous state, so use the latest estimates.
        //
        // Timestamp of previous value.
        long t0us = m.getKey();
        // Keys for the previous timestamp.
        Key x0 = Keys.poseKey(t0us);
        Key b0 = Keys.biasKey(t0us);

        // Previous estimates
        Pose2 pose0 = m_result.atPose2(x0);
        double bias0 = m_result.atDouble(b0);

        // Use the previous estimates as the initial values
        newValues.insert(x1, pose0);
        newValues.insert(b1, bias0);

        // Also add prior factors with these same values, to limit variation.
        // if the model does the wrong thing these priors will also make it
        // "stick" longer.
        // Priors.poseSmoother(factors, x1, pose0);
        // Priors.biasSmoother(factors, b0, bias0);

        // Measurement at previous time.
        OdoGyroMeasurement m0 = m.getValue();

        // Factor linking the previous odometry-derived pose to the new pose.
        factors.add(getOdometryFactor(m_kinematics, x0, x1, m0.odo(), pos1));

        // Factor linking the previous yaw to the new yaw.
        Rot2 dr = new Rot2(yaw1.minus(m0.gyro()).getRadians());
        double dt = (t1us - t0us) / 1e6;
        factors.add(PlanarGyroFactor.FromRotation(x0, x1, b0, m_params, dr, dt));
        factors.add(PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor(
                b0, b1, m_params));
    }

}
