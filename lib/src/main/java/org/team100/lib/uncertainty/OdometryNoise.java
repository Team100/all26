package org.team100.lib.uncertainty;

/**
 * Methods governing odometry update uncertainties
 */
public class OdometryNoise {

    public static IsotropicNoiseSE2 get(double distanceM, double rotationRad) {
        double cartesian = cartesian(distanceM);
        double rotation = rotation(distanceM, rotationRad);
        return IsotropicNoiseSE2.fromStdDev(cartesian, rotation);
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
     * 
     * TODO: calibrate this
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
