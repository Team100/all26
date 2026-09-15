package org.team100.lib.gtsam.simulation;

import java.util.List;
import java.util.Random;

import org.wpilib.math.geometry.Pose2d;

import org.team100.lib.gtsam.field.FieldMap;
import gtsam.Pose2;
import gtsam.Vector3;
import org.team100.lib.gtsam.kinodynamics.Kinematics;
import org.team100.lib.gtsam.kinodynamics.Kinematics.PointR2;
import org.team100.lib.gtsam.kinodynamics.Kinematics.Twist2d;
import org.team100.lib.gtsam.util.Geometry;

/**
 * Computes odometry based on robot pose.
 */
public class SimulatedOdometry {
    private static Random RANDOM = new Random(42);

    private final Kinematics.SwerveDriveKinematics100 kinematics;

    /** Previous positions. */
    private Kinematics.SwerveModulePositions positions;
    /** Previous pose */
    private Pose2 pose;

    public SimulatedOdometry(FieldMap fieldMap, Pose2d initial) throws Throwable {
        kinematics = new Kinematics.SwerveDriveKinematics100(
                List.of(
                        new PointR2(0.5, 0.5),
                        new PointR2(0.5, -0.5),
                        new PointR2(-0.5, 0.5),
                        new PointR2(-0.5, -0.5)));
        // Positions start at zero.
        positions = new Kinematics.SwerveModulePositions(
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)));
        pose = Geometry.toPose2(initial);
    }

    /**
     * Uses the previous given pose to compute new module positions for the new
     * pose. Adds 1% noise.
     * 
     * @param gtPose2 current ground-truth pose.
     */
    public Kinematics.SwerveModulePositions positions(
            Pose2d gtPose2d) throws Throwable {
        final Pose2 newPose = Geometry.toPose2(gtPose2d);

        // twist from previous "pose" to new "newPose"
        Vector3 twist = pose.logmap(newPose);
        pose = newPose;

        Vector3 tNoise = new Vector3(
                noise(twist.at(0)), noise(twist.at(1)), noise(twist.at(2)));
        Twist2d twist2d = Twist2d.fromVector(twist.plus(tNoise));
        // transform the twist to update the module positions.
        positions = kinematics.to_swerve_module_positions(positions, twist2d);
        return positions;
    }

    private double noise(double t) {
        return zero(t);
        // return onePercent(t);
    }

    private double zero(double t) {
        return 0;
    }

    /** 1% noise */
    private double onePercent(double t) {
        return t * RANDOM.nextGaussian(0, 0.01);
    }
}
