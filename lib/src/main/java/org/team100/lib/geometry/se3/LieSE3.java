package org.team100.lib.geometry.se3;

import org.team100.lib.geometry.GeometryUtil;
import org.wpilib.math.geometry.Twist3d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N4;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;

/**
 * See https://arxiv.org/pdf/1812.01537.
 * 
 * There are also useful examples in Sophus,
 * https://github.com/strasdat/Sophus/blob/main/sophus/se3.hpp
 */
public class LieSE3 {
    /**
     * Map an R6 vector, tangent to SE(3), to the Lie algebra, se(3),
     * which is a 4x4 matrix:
     * 
     * | 0 -rz ry dx |
     * | rz 0 -rx dy |
     * | -ry rx 0 dx |
     * | 0 0 0 0 |
     */
    public static Matrix<N4, N4> hat(Twist3d t) {
        return MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0.00, -t.rz, t.ry, t.dx, //
                t.rz, 0.00, -t.rx, t.dy, //
                -t.ry, t.rx, 0.00, t.dz, //
                0.00, 0.00, 0.00, 0.00);
    }

    public static Matrix<N4, N4> hat(Vector<N6> t) {
        return MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0.00, -t.get(5), t.get(4), t.get(0), //
                t.get(5), 0.00, -t.get(3), t.get(1), //
                -t.get(4), t.get(3), 0.00, t.get(2), //
                0.00, 0.00, 0.00, 0.00);
    }

    /**
     * Map a member of the Lie algebra, se(3), to the tangent space, R6.
     * 
     * @param m should have the se(3) structure (this does not check)
     */
    public static Twist3d vee(Matrix<N4, N4> m) {
        return new Twist3d(
                m.get(0, 3),
                m.get(1, 3),
                m.get(2, 3),
                m.get(2, 1),
                m.get(0, 2),
                m.get(1, 0));
    }

    /** The bracket of two tangents [A, B] which is AB - BA. */
    public static Twist3d bracket(Twist3d a, Twist3d b) {
        Matrix<N4, N4> ahat = hat(a);
        Matrix<N4, N4> bhat = hat(b);
        Matrix<N4, N4> b1 = ahat.times(bhat);
        Matrix<N4, N4> b2 = bhat.times(ahat);
        Matrix<N4, N4> s = b1.minus(b2);
        return vee(s);
    }

    /** The bracket of two tangents [A, B] which is AB - BA. */
    public static Vector<N6> bracket(Vector<N6> a, Vector<N6> b) {
        Matrix<N4, N4> ahat = hat(a);
        Matrix<N4, N4> bhat = hat(b);
        Matrix<N4, N4> b1 = ahat.times(bhat);
        Matrix<N4, N4> b2 = bhat.times(ahat);
        Matrix<N4, N4> s = b1.minus(b2);
        return GeometryUtil.toVec(vee(s));
    }

}
