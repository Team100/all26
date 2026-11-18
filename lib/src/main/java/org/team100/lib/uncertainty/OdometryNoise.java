package org.team100.lib.uncertainty;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.math.util.Nat;

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

    public static IsotropicNoiseSE2 get(Twist2d twist) {
        double odometryDistanceM = Metrics.translationalNorm(twist);
        double odometryRotationRad = twist.dtheta;
        return get(odometryDistanceM, odometryRotationRad);
    }

    /**
     * Use the old cartesian and rotation terms but add correlation.
     * 
     * This is a wild guess.
     */
    public static AnisotropicNoiseSE2 getFull(Twist2d twist) {
        double l2 = Metrics.l2Norm(twist);
        if (l2 < 1e-6)
            return new AnisotropicNoiseSE2();
        double rad = twist.dtheta;
        double cartesian = cartesian(l2);
        double rotation = rotation(l2, rad);
        double c = twist.dx / l2;
        double s = twist.dy / l2;
        double r = twist.dtheta / l2;
        // note regularization terms
        double w00 = cartesian * c + 1e-6;
        double w11 = cartesian * s + 1e-6;
        double w22 = rotation * r + 1e-6;
        // correlation is a guess
        double p = 0.75;
        return new AnisotropicNoiseSE2(
                MatBuilder.fill(Nat.N3(), Nat.N3(),
                        w00 * w00, p * w00 * w11, p * w00 * w22, //
                        p * w00 * w11, w11 * w11, p * w11 * w22, //
                        p * w00 * w22, p * w11 * w22, w22 * w22));
    }

    /**
     * Use wheel noise and jacobians to derive twist noise.
     */
    public static AnisotropicNoiseSE2 get(
            SwerveDriveKinematics100 k,
            SwerveModuleDeltas deltas) {
        Matrix<N8, N8> wheelNoise = allWheelNoise(deltas);
        Matrix<N8, N8> wheelJ = allWheelJ(deltas);
        Matrix<N8, N8> cornerNoise = wheelJ.times(wheelNoise).times(wheelJ.transpose());

        Matrix<N3, N8> driveJ = k.J();
        Matrix<N3, N3> twistNoise = driveJ.times(cornerNoise).times(driveJ.transpose());
        // regularization
        twistNoise = twistNoise.plus(Matrix.eye(Nat.N3()).times(1e-9));
        return new AnisotropicNoiseSE2(twistNoise);
    }

    static IsotropicNoiseSE2 get(double distanceM, double rotationRad) {
        double cartesian = cartesian(distanceM);
        double rotation = rotation(distanceM, rotationRad);
        return IsotropicNoiseSE2.fromStdDev(cartesian, rotation);
    }

    /**
     * Noise of all the wheels together.
     * 
     * This describes "noise" meaning "difference from actual".
     * 
     * The wheel sensor noises, per se, are uncorrelated but
     * the actual errors are very much correlated.
     * 
     * Error would increase upon acceleration, but that effect
     * is not included here.
     */
    private static Matrix<N8, N8> allWheelNoise(SwerveModuleDeltas d) {
        SwerveModuleDelta[] all = d.all();
        double l0 = cartesian(all[0].distanceMeters());
        double a0 = 0.02;
        double l1 = cartesian(all[1].distanceMeters());
        double a1 = 0.02;
        double l2 = cartesian(all[2].distanceMeters());
        double a2 = 0.02;
        double l3 = cartesian(all[3].distanceMeters());
        double a3 = 0.02;
        // guess at correlation.
        double p = 0.75;
        return MatBuilder.fill(Nat.N8(), Nat.N8(), //
                l0 * l0, 0, p * l0 * l1, 0, p * l0 * l2, 0, p * l0 * l3, 0, //
                0, a0 * a0, 0, p * a0 * a1, 0, p * a0 * a2, 0, p * a0 * a3, //
                p * l1 * l0, 0, l1 * l1, 0, p * l1 * l2, 0, p * l1 * l3, 0, //
                0, p * a1 * a0, 0, a1 * a1, 0, p * a1 * a2, 0, p * a1 * a3, //
                p * l2 * l0, 0, p * l2 * l1, 0, l2 * l2, 0, p * l2 * l3, 0, //
                0, p * a2 * a0, 0, p * a2 * a1, 0, a2 * a2, 0, p * a2 * a3, //
                p * l3 * l0, 0, p * l3 * l1, 0, p * l3 * l2, 0, l3 * l3, 0, //
                0, p * a3 * a0, 0, p * a3 * a1, 0, p * a3 * a2, 0, a3 * a3);
    }

    /**
     * Jacobian of the polar transforms from wheel [r,theta] to
     * corner [dx, dy], all together.
     */
    private static Matrix<N8, N8> allWheelJ(SwerveModuleDeltas d) {
        SwerveModuleDelta[] all = d.all();
        double r0 = all[0].distanceMeters();
        double r1 = all[1].distanceMeters();
        double r2 = all[2].distanceMeters();
        double r3 = all[3].distanceMeters();
        double theta0 = all[0].wrappedAngle().orElse(Rotation2d.ZERO).getRadians();
        double theta1 = all[1].wrappedAngle().orElse(Rotation2d.ZERO).getRadians();
        double theta2 = all[2].wrappedAngle().orElse(Rotation2d.ZERO).getRadians();
        double theta3 = all[3].wrappedAngle().orElse(Rotation2d.ZERO).getRadians();
        double c0 = Math.cos(theta0);
        double s0 = Math.sin(theta0);
        double c1 = Math.cos(theta1);
        double s1 = Math.sin(theta1);
        double c2 = Math.cos(theta2);
        double s2 = Math.sin(theta2);
        double c3 = Math.cos(theta3);
        double s3 = Math.sin(theta3);
        return MatBuilder.fill(Nat.N8(), Nat.N8(), //
                c0, -r0 * s0, 0, 0, 0, 0, 0, 0, //
                s0, r0 * c0, 0, 0, 0, 0, 0, 0, //
                0, 0, c1, -r1 * s1, 0, 0, 0, 0, //
                0, 0, s1, r1 * c1, 0, 0, 0, 0, //
                0, 0, 0, 0, c2, -r2 * s2, 0, 0, //
                0, 0, 0, 0, s2, r2 * c2, 0, 0, //
                0, 0, 0, 0, 0, 0, c3, -r3 * s3, //
                0, 0, 0, 0, 0, 0, s3, r3 * c3);
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
    static double cartesian(double distanceM) {
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
