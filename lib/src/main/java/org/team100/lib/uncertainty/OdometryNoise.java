package org.team100.lib.uncertainty;

import org.team100.lib.geometry.Metrics;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Uncertainty estimates for odometry, using kinda wild guesses.
 * 
 * Noise is generally zero when motionless, and superlinear with speed.
 * 
 * Note this isn't really *noise*, this is measurement error, which I think is
 * actually unexplained bias. In 2024, we measured very carefully and got 5%
 * bias in one direction, but the opposite in the opposite direction (so a
 * round-trip wouldn't end up in the same place). We never figured out what the
 * cause was, so, just call it 5% error. :-(
 */
public class OdometryNoise {

    public static IsotropicNoiseSE2 get(double distanceM, double rotationRad) {
        double cartesian = cartesian(distanceM);
        double rotation = rotation(distanceM, rotationRad);
        return IsotropicNoiseSE2.fromStdDev(cartesian, rotation);
    }

    public static IsotropicNoiseSE2 get(Twist2d twist) {
        double odometryDistanceM = Metrics.translationalNorm(twist);
        double odometryRotationRad = twist.dtheta;
        return get(odometryDistanceM, odometryRotationRad);
    }

    /**
     * The error in odometry is superlinear in speed. Since the odometry samples
     * happen regularly, we can use the sample distance as a measure of speed.
     * 
     * This yields zero when the robot isn't moving, which is what you'd expect.
     * 
     * I completely made this up
     * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=995645441#gid=995645441
     */
    private static double cartesian(double distanceM) {
        double norm = Math.abs(distanceM);
        // We kinda measured 5% error in the best (slow) case, in 2024.
        double lowSpeedError = 0.05;
        // This is just a guess
        double superError = 0.5;
        return lowSpeedError * norm + superError * norm * norm;
    }

    /**
     * How does rotation error scale with speed? Driving in a straight line
     * definitely produces rotational drift, so this isn't just proportional to the
     * odometry rotation term alone.
     * 
     * Maybe just add them, 1 meter == 1 radian.
     */
    private static double rotation(double distanceM, double rotationRad) {
        double norm = Math.abs(distanceM) + Math.abs(rotationRad);
        // We kinda measured 5% error in the best (slow) case.
        // I boosted this to help rely more on the gyro
        // double lowSpeedError = 0.05;
        double lowSpeedError = 0.25;
        // This is just a guess
        double superError = 0.5;
        // We haven't measured this, so just guess it's the same???
        return lowSpeedError * norm + superError * norm * norm;
    }
}
