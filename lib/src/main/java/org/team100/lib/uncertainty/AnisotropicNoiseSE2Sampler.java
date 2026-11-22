package org.team100.lib.uncertainty;

import java.util.Random;

import org.team100.lib.geometry.GeometryUtil;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

public class AnisotropicNoiseSE2Sampler {
    private final Random r = new Random();

    public Twist2d apply(AnisotropicNoiseSE2 m_noise, Twist2d t) {
        Matrix<N3, N3> llt = m_noise.cov().times(10000).lltDecompose(true).div(100);
        Vector<N3> mean = GeometryUtil.toVec(t);
        Vector<N3> z = VecBuilder.fill(r.nextGaussian(), r.nextGaussian(), r.nextGaussian());
        Vector<N3> Lz = new Vector<>(llt.times(z));
        Vector<N3> sampleVec = mean.plus(Lz);
        return GeometryUtil.fromVec(sampleVec);
    }
}
