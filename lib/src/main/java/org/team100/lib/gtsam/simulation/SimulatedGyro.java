package org.team100.lib.gtsam.simulation;

import java.util.Random;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class SimulatedGyro {
    private static Random RANDOM = new Random(42);
    // white noise in yaw. note this is not the correct noise spectrum.
    private static final double NOISE = 1e-5;
    // drift rate in rad/s
    private final double m_drift;

    public SimulatedGyro(boolean drift) {
        if (drift) {
            m_drift = 0;
            // m_drift = 1e-3;
        } else {
            m_drift = 0;
        }
    }

    public double yaw(double t_us, Pose2d gtPose2d) {
        return gt(t_us, gtPose2d);
        // return noisy(t_us, gtPose2d);
    }

    private double gt(double t_us, Pose2d gtPose2d) {
        return gtPose2d.getRotation().getRadians();
    }

    private double noisy(double t_us, Pose2d gtPose2d) {
        Rotation2d gtRot = gtPose2d.getRotation();
        Rotation2d driftRot = new Rotation2d(m_drift * t_us * 1e-6);
        Rotation2d noiseRot = new Rotation2d(RANDOM.nextGaussian(0, NOISE));
        Rotation2d totalRot = gtRot.plus(driftRot).plus(noiseRot);
        return totalRot.getRadians();
    }

}
