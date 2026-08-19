package org.team100.lib.geometry.se2;

import org.team100.lib.geometry.GeometryUtil;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Nat;

/**
 * See https://arxiv.org/pdf/1812.01537, particularly appendix C.
 * 
 * There are also useful examples in Sophus,
 * https://github.com/strasdat/Sophus/blob/main/sophus/se2.hpp
 */
public class LieSE2 {

    /**
     * Map an R3 vector, tangent to SE(2), to the Lie algebra, se(2),
     * which is a 3x3 matrix:
     * 
     * | 0 -θ x |
     * | θ 0 y |
     * | 0 0 0 \
     */
    public static Matrix<N3, N3> hat(Twist2d t) {
        return MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -t.dtheta, t.dx, //
                t.dtheta, 0, t.dy, //
                0, 0, 0);
    }

    public static Matrix<N3, N3> hat(Vector<N3> t) {
        return MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -t.get(2), t.get(0), //
                t.get(2), 0, t.get(1), //
                0, 0, 0);
    }

    /**
     * Map a member of the Lie algebra, se(2), to the tangent space, R3.
     * 
     * @param m should have the se(2) structure (this does not check).
     */
    public static Twist2d vee(Matrix<N3, N3> m) {
        return new Twist2d(m.get(0, 2), m.get(1, 2), m.get(1, 0));
    }

    /**
     * The bracket of two tangents [A, B], which is AB - BA.
     * 
     * There's a useful interpretation of the Lie bracket on page 159
     * of Spivak:
     * https://dl.icdst.org/pdfs/files3/e9091aa2ddcfcbf04faeb46c68d7dc49.pdf
     */
    public static Twist2d bracket(Twist2d a, Twist2d b) {
        Matrix<N3, N3> ahat = hat(a);
        Matrix<N3, N3> bhat = hat(b);
        Matrix<N3, N3> b1 = ahat.times(bhat);
        Matrix<N3, N3> b2 = bhat.times(ahat);
        Matrix<N3, N3> s = b1.minus(b2);
        return vee(s);
    }

    /** vector version of bracket */
    public static Vector<N3> bracket(Vector<N3> a, Vector<N3> b) {
        Matrix<N3, N3> ahat = hat(a);
        Matrix<N3, N3> bhat = hat(b);
        Matrix<N3, N3> b1 = ahat.times(bhat);
        Matrix<N3, N3> b2 = bhat.times(ahat);
        Matrix<N3, N3> s = b1.minus(b2);
        return GeometryUtil.toVec(vee(s));
    }
}
