package org.team100.lib.gtsam.pose_estimator;

import java.util.List;

import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.Pose2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Base;
import gtsam.noiseModel.Diagonal;
import org.team100.lib.gtsam.kinodynamics.DriveUtil;
import org.team100.lib.gtsam.kinodynamics.Kinematics;
import org.team100.lib.gtsam.kinodynamics.Kinematics.PointR2;

/**
 * Odometry uses "betweeen" factors to represent the difference between poses
 * derived from the drive module positions.
 */
public class Odometry {
    private final Solver estimate;
    private final Kinematics.SwerveDriveKinematics100 kinematics;

    private Kinematics.SwerveModulePositions positions;
    private Long t0_us = null;

    public Odometry(Solver e) throws Throwable {
        estimate = e;
        kinematics = new Kinematics.SwerveDriveKinematics100(
                List.of(
                        new PointR2(0.5, 0.5),
                        new PointR2(0.5, -0.5),
                        new PointR2(-0.5, 0.5),
                        new PointR2(-0.5, -0.5)));

        positions = new Kinematics.SwerveModulePositions(
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)),
                new Kinematics.SwerveModulePosition100(
                        0, new Kinematics.RotR2(1, 0)));
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
            Kinematics.SwerveModulePositions newPositions) throws Throwable {

        if (t0_us == null) {
            this.positions = newPositions;
            t0_us = t1_us;
            return;
        }

        Kinematics.SwerveModuleDeltas deltas = DriveUtil.module_position_delta(
                positions, newPositions);

        // Tangent-space (twist) measurement.
        Kinematics.Twist2d twist = kinematics.to_twist_2d(deltas);
        // Twist as a GTSAM vector.
        Vector3 twistVector = new Vector3(
                twist.x(),
                twist.y(),
                twist.theta());
        // Factor measurement.
        Pose2 measurement = new Pose2().expmap(twistVector);

        // The thing "between" two poses is another pose, not a
        // twist:
        //
        // pose1.compose(betweenpose) = pose2.
        estimate.add(BetweenFactorPose2.newBetweenFactorPose2(
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
