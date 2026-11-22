package org.team100.lib.uncertainty;

import java.util.Random;

import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

public class IsotropicNoiseSE2Sampler {
    private final IsotropicNoiseSE2 m_noise;
    private final Random m_rand;

    public IsotropicNoiseSE2Sampler(IsotropicNoiseSE2 noise) {
        m_noise = noise;
        m_rand = new Random();
    }

    public Twist2d apply(Twist2d t) {
        Vector<N3> w = m_noise.sample(m_rand);
        return new Twist2d(
                t.dx + w.get(0),
                t.dy + w.get(1),
                t.dtheta + w.get(2));
    }
}
