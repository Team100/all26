package org.team100.lib.gtsam.pose_estimator;

import java.util.Optional;

import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.Pose2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * Odometry uses "betweeen" factors to represent the difference between poses
 * derived from the drive module positions.
 */
public class Odometry {
    private static final boolean DEBUG = false;
    private final Solver solver;
    private final SwerveDriveKinematics100 kinematics;

    private SwerveModulePositions positions;
    private Long t0_us = null;

    public Odometry(Solver e) throws Throwable {
        solver = e;
        kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        positions = new SwerveModulePositions(
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))),
                new SwerveModulePosition100(
                        0, Optional.of(new Rotation2d(1, 0))));
    }

    /**
     * Add an odometry measurement using a "between" factor.
     * 
     * Remember to call addVariable so that the odometry factor has something to
     * refer to.
     * 
     * @param t1_us timestamp in microseconds.
     */
    public void add(
            long t1_us,
            SwerveModulePositions newPositions) throws Throwable {
        if (DEBUG)
            System.out.printf("add odometry factor %d\n", t1_us);

        if (t0_us == null) {
            this.positions = newPositions;
            t0_us = t1_us;
            return;
        }

        SwerveModuleDeltas deltas = SwerveModuleDeltas.modulePositionDelta(
                positions, newPositions);

        // Tangent-space (twist) measurement.
        Twist2d twist = kinematics.forward(deltas);
        // Twist as a GTSAM vector.
        Vector3 twistVector = new Vector3(
                twist.dx,
                twist.dy,
                twist.dtheta);
        // Factor measurement.
        Pose2 measurement = new Pose2().expmap(twistVector);

        // The thing "between" two poses is another pose, not a
        // twist:
        //
        // pose1.compose(betweenpose) = pose2.
        solver.add(BetweenFactorPose2.newBetweenFactorPose2(
                Key.X(t0_us), Key.X(t1_us), measurement, noise(twistVector)));

        positions = newPositions;
        t0_us = t1_us;
    }

    /**  */
    private shared_ptr<Diagonal> noise(Vector3 twist) throws Throwable {
        double distance = twist.norm();
        return Diagonal.Sigmas(
                new Vector3(
                        noise(distance),
                        noise(distance),
                        noise(distance)));
    }

    /** Speed-dependent noise. */
    private double noise(double distance) {
        return 0.0001 + 0.02 * distance;
    }

}
