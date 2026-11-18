package org.team100.lib.uncertainty;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N8;
import org.wpilib.math.util.Nat;

/**
 * This is to explore non-isotropic noise models, making scatter plots of
 * samples.
 * 
 * https://docs.google.com/spreadsheets/d/12WoEP8JOxmGjk9os07973DPKS-xjhoHNlxL0YbhnToA
 */
public class NoiseTest {
    private static final boolean DEBUG = false;
    SwerveDriveKinematics100 k = new SwerveDriveKinematics100(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    /**
     * This shows the isotropic noise around the twist endpoint,
     * which is definitely wrong.
     */
    @Test
    void testOdoNoise() {
        // the maximum speed is 5 m/s, the clock is 0.02 sec, so the
        // maximum twist is 0.1 m.
        Twist2d t = new Twist2d(0.07, 0.07, 0);
        IsotropicNoiseSE2 noise = OdometryNoise.get(t);
        IsotropicNoiseSE2Sampler sampler = new IsotropicNoiseSE2Sampler(noise);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    /**
     * This shows the noise is mostly collinear with the twist direction, which is
     * more correct.
     */
    @Test
    void testAnisotropicOdoNoise() {
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = new Twist2d(0.07, 0.07, 0);
        AnisotropicNoiseSE2 noise = OdometryNoise.getFull(t);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    /**
     * Use wheel noise estimates and kinematics.
     */
    @Test
    void testNoiseFromJacobian() {
        // say each wheel is the same, 0.1m displacement,
        // pi/4 angle.

        double r = 0.1;
        double theta = Math.PI / 4;

        SwerveModuleDelta delta = new SwerveModuleDelta(r, Optional.of(new Rotation2d(theta)));
        SwerveModuleDeltas deltas = new SwerveModuleDeltas(delta, delta, delta, delta);

        double c = Math.cos(theta);
        double s = Math.sin(theta);
        Matrix<N2, N2> wheelJ = MatBuilder.fill(Nat.N2(), Nat.N2(), //
                c, -r * s, //
                s, r * c);
        if (DEBUG)
            System.out.println("WHEEL J");
        if (DEBUG)
            System.out.println(wheelJ);
        // use the linear noise from the previous model
        double lin = OdometryNoise.cartesian(r);
        // say the angle noise is a constant, like 0.02 rad
        double ang = 0.02;
        // there's no correlation between the two.
        // this is (r, theta) variances.
        Matrix<N2, N2> wheelNoise = MatBuilder.fill(Nat.N2(), Nat.N2(), //
                lin * lin, 0, //
                0, ang * ang);
        if (DEBUG)
            System.out.println("WHEEL NOISE");
        if (DEBUG)
            System.out.println(wheelNoise);

        Matrix<N2, N2> cornerNoise = wheelJ.times(wheelNoise).times(wheelJ.transpose());
        if (DEBUG)
            System.out.println("CORNER NOISE");
        if (DEBUG)
            System.out.println(cornerNoise);

        // forard kinematics of the drivetrain
        Matrix<N3, N8> driveJ = k.J();
        if (DEBUG)
            System.out.println("DRIVE J");
        if (DEBUG)
            System.out.println(driveJ);

        // fl, fr, rl, rr but all same anyway
        Matrix<N8, N8> allCornerNoise = new Matrix<>(Nat.N8(), Nat.N8());
        for (int i = 0; i < 7; i += 2) {
            for (int j = 0; j < 7; j += 2) {
                // guess at correlation
                double rho = (i == j) ? 1.0 : 0.75;
                allCornerNoise.assignBlock(i, j, cornerNoise.times(rho));
            }
        }
        if (DEBUG)
            System.out.println("ALL CORNERS");
        if (DEBUG)
            System.out.println(allCornerNoise);

        Matrix<N3, N3> twistNoise = driveJ.times(allCornerNoise).times(driveJ.transpose());
        // regularization
        twistNoise = twistNoise.plus(Matrix.eye(Nat.N3()).times(1e-9));
        if (DEBUG)
            System.out.println("TWIST NOISE");
        if (DEBUG)
            System.out.println(twistNoise);

        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = k.forward(deltas);
        AnisotropicNoiseSE2 noise = new AnisotropicNoiseSE2(twistNoise);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }

    }

    @Test
    void testWheels2() {
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = new Twist2d(0.07, 0.07, 0);
        SwerveModuleDeltas d = k.inverse(t);
        AnisotropicNoiseSE2 noise = OdometryNoise.get(k, d);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    @Test
    void testWheels2Turning() {
        // turning very fast indeed
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = new Twist2d(0.07, 0.07, 0.2);
        SwerveModuleDeltas d = k.inverse(t);
        AnisotropicNoiseSE2 noise = OdometryNoise.get(k, d);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    @Test
    void testAtRest() {
        // at rest, the odometry noise is nearly zero.
        // note that the GTSAM noise *model* should be
        // softer than this, or the vision input
        // won't have any leverage.
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = new Twist2d(0, 0, 0);
        SwerveModuleDeltas d = k.inverse(t);
        AnisotropicNoiseSE2 noise = OdometryNoise.get(k, d);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    @Test
    void testTurnInPlace() {
        // the maximum turning rate is very fast,
        // something like 20 rad/s
        // this produces an enormous amount of noise,
        // centered at the origin.
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler();
        Twist2d t = new Twist2d(0, 0, 0.4);
        SwerveModuleDeltas d = k.inverse(t);
        AnisotropicNoiseSE2 noise = OdometryNoise.get(k, d);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(noise, t);
            if (DEBUG)
                System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }
}
