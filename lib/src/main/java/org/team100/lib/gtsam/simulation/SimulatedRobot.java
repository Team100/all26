package org.team100.lib.gtsam.simulation;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Computes a deterministic pose based on the given time.
 * 
 * The robot drives in a circle while panning back and forth.
 */
public class SimulatedRobot {
    /** x center of the circle, meters */
    private static double CX = 4;
    /** y center of the circle, meters */
    private static double CY = 4;
    /** circle radius, meters */
    private static double RADIUS = 2;
    private static double PATH_PERIOD_S = 2.0 * Math.PI;
    private static double PAN_PERIOD_S = PATH_PERIOD_S / 3;
    // maximum pan angle, radians
    private static double PAN_SCALE_RAD = 0.5;

    public SimulatedRobot() {
    }

    /**
     * @param time_s sim time in seconds
     * @return robot pose
     */
    public Pose2d pose(long t1_us) {
        return justCircle(t1_us);
        // return justPan(t1_us);
        // return stationary(t1_us);
        // return circleAndPan(t1_us);
    }

    private Pose2d justCircle(long t1_us) {
        double time_s = (double) t1_us * 1e-6;
        double angle = 2 * Math.PI * time_s / PATH_PERIOD_S;
        double gt_x = CX + RADIUS * Math.cos(angle);
        double gt_y = CY + RADIUS * Math.sin(angle);
        return new Pose2d(gt_x, gt_y, new Rotation2d());
    }

    private Pose2d justPan(long t1_us) {
        double time_s = (double) t1_us * 1e-6;
        double gt_theta = PAN_SCALE_RAD * Math.sin(
                2 * Math.PI * time_s / PAN_PERIOD_S);
        return new Pose2d(CX, CY, new Rotation2d(gt_theta));
    }

    private Pose2d circleAndPan(long t1_us) {
        double time_s = (double) t1_us * 1e-6;
        double angle = 2 * Math.PI * time_s / PATH_PERIOD_S;
        double gt_x = CX + RADIUS * Math.cos(angle);
        double gt_y = CY + RADIUS * Math.sin(angle);
        double gt_theta = PAN_SCALE_RAD * Math.sin(
                2 * Math.PI * time_s / PAN_PERIOD_S);
        return new Pose2d(gt_x, gt_y, new Rotation2d(gt_theta));
    }

    private Pose2d stationary(long t1_us) {
        return new Pose2d(CX, CY, new Rotation2d());
    }

}
