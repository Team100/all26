package org.team100.lib.uncertainty;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Twist2d;

/**
 * This is to explore non-isotropic noise models, making scatter plots of
 * samples.
 * 
 * https://docs.google.com/spreadsheets/d/12WoEP8JOxmGjk9os07973DPKS-xjhoHNlxL0YbhnToA
 */
public class NoiseTest {
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
            System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }

    /**
     * This shows the noise is mostly collinear with the twist direction, which is
     * more correct.
     */
    @Test
    void testAnisotropicOdoNoise() {
        Twist2d t = new Twist2d(0.07, 0.07, 0);
        AnisotropicNoiseSE2 noise = OdometryNoise.getFull(t);
        AnisotropicNoiseSE2Sampler sampler = new AnisotropicNoiseSE2Sampler(noise);
        for (int i = 0; i < 50; ++i) {
            Twist2d n = sampler.apply(t);
            System.out.printf("%f, %f\n", n.dx, n.dy);
        }
    }
}
