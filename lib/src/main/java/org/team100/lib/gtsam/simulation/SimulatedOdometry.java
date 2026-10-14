package org.team100.lib.gtsam.simulation;

import java.util.Optional;
import java.util.Random;

import org.team100.lib.gtsam.field.FieldMap;
import org.team100.lib.gtsam.util.Geometry;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.Pose2;
import gtsam.Vector3;

/**
 * Computes odometry based on robot pose.
 */
public class SimulatedOdometry {
    private static Random RANDOM = new Random(42);

    private final SwerveDriveKinematics100 kinematics;

    /** Previous positions. */
    private SwerveModulePositions positions;
    /** Previous pose */
    private Pose2 pose;

    public SimulatedOdometry(FieldMap fieldMap, Pose2d initial) throws Throwable {
        kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // Positions start at zero.
        positions = new SwerveModulePositions(
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))));
        pose = Geometry.toPose2(initial);
    }

    /**
     * Uses the previous given pose to compute new module positions for the new
     * pose. Adds 1% noise.
     * 
     * @param gtPose2 current ground-truth pose.
     */
    public SwerveModulePositions positions(
            Pose2d gtPose2d) throws Throwable {
        final Pose2 newPose = Geometry.toPose2(gtPose2d);

        // twist from previous "pose" to new "newPose"
        Vector3 twist = pose.logmap(newPose);
        pose = newPose;

        Vector3 tNoise = new Vector3(
                noise(twist.at(0)), noise(twist.at(1)), noise(twist.at(2)));
        Twist2d twist2d = Geometry.twistFromVector(twist.plus(tNoise));
        // transform the twist to update the module positions.
        SwerveModuleDeltas d = kinematics.inverse(twist2d);
        positions = SwerveModulePositions.modulePositionFromDelta(positions, d);
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
