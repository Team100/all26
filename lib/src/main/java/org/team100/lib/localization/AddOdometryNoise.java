package org.team100.lib.localization;

import java.util.Random;
import java.util.function.UnaryOperator;

import org.team100.lib.geometry.Metrics;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.OdometryNoise;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Add noise to the twist computed from wheel deltas, for simulation
 * 
 * This noise source is not very realistic, but it's better than independent
 * noise at each wheel.
 */
public class AddOdometryNoise implements UnaryOperator<Twist2d> {
    private final Random m_rand;

    public AddOdometryNoise() {
        m_rand = new Random();
    }

    public Twist2d apply(Twist2d x) {
        double distanceM = Metrics.translationalNorm(x);
        IsotropicNoiseSE2 noise = OdometryNoise.get(distanceM, x.dtheta);
        return new Twist2d(
                x.dx + noise.cartesian() * m_rand.nextGaussian(),
                x.dy + noise.cartesian() * m_rand.nextGaussian(),
                x.dtheta + noise.rotation() * m_rand.nextGaussian());
    }

}
