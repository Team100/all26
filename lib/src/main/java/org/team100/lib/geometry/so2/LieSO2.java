package org.team100.lib.geometry.so2;

import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

/**
 * See https://arxiv.org/pdf/1812.01537
 * 
 * There are also useful examples in Sophus,
 * https://github.com/strasdat/Sophus/blob/main/sophus/so2.hpp
 */
public class LieSO2 {
    /**
     * Map a scalar, tangent to SO(2), to the Lie algebra, so(2).
     * 
     * | 0 -θ |
     * | θ 0 |
     * 
     * @return skew-symmetric matrix representing theta.
     */
    public static Matrix<N2, N2> hat(double theta) {
        return MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, -theta, //
                theta, 0);
    }

    /**
     * Map a member of the Lie algebra, so(2), to the tangent space, R1.
     * 
     * @param m should be skew-symmetric (this does not check).
     */
    public static double vee(Matrix<N2, N2> m) {
        return m.get(1, 0);
    }

    /**
     * The bracket of two tangents [a, b].
     * 
     * In SO(2) the bracket is always zero because SO(2) is abelian (commutative).
     */
    public static double bracket(double a, double b) {
        return 0;
    }

}
