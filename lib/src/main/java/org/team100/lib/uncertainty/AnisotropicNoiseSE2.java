package org.team100.lib.uncertainty;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

/**
 * Uncertainty in SE(2) with nonzero covariance, represented by a 3x3 covariance
 * matrix.
 */
public class AnisotropicNoiseSE2 {
    private final Matrix<N3, N3> m_cov;

    public AnisotropicNoiseSE2() {
        this(new Matrix<>(Nat.N3(), Nat.N3()));
    }

    public AnisotropicNoiseSE2(Matrix<N3, N3> cov) {
        m_cov = cov;
    }

    public Matrix<N3, N3> cov() {
        return m_cov;
    }

}
