package org.team100.lib.localization.simulation;

import java.util.Random;
import java.util.function.Function;

import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.OdometryNoise;
import org.wpilib.math.geometry.Twist2d;

/**
 * Add noise to the twist computed from wheel deltas, for simulation
 * 
 * This noise source is not very realistic, but it's better than independent
 * noise at each wheel.
 */
public class AddOdometryNoise implements Function<SwerveModuleDeltas, Twist2d> {
    private final SwerveDriveKinematics100 m_k;

    private final Random m_rand;

    public AddOdometryNoise(SwerveDriveKinematics100 k) {
        m_k = k;
        m_rand = new Random();
    }

    public Twist2d apply(SwerveModuleDeltas d) {
        Twist2d x = m_k.forward(d);
        IsotropicNoiseSE2 noise = OdometryNoise.get(x);
        return new Twist2d(
                x.dx + noise.cartesian() * m_rand.nextGaussian(),
                x.dy + noise.cartesian() * m_rand.nextGaussian(),
                x.dtheta + noise.rotation() * m_rand.nextGaussian());
    }

}
