package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

/**
 * See modern_robotics/core.py
 */
public class ModernRoboticsTest {

    // Example Input (3 Link Robot)
    Vector<N3> thetalist = VecBuilder.fill(0.1, 0.1, 0.1);
    Vector<N3> dthetalist = VecBuilder.fill(0.1, 0.2, 0.3);
    Vector<N3> ddthetalist = VecBuilder.fill(2, 1.5, 1);
    Vector<N3> taulist = VecBuilder.fill(0.5, 0.6, 0.7);
    Vector<N3> g = VecBuilder.fill(0, 0, -9.8);
    Vector<N6> Ftip = VecBuilder.fill(1, 1, 1, 1, 1, 1);
    Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
            1, 0, 0, 0, //
            0, 1, 0, 0, //
            0, 0, 1, 0.089159, //
            0, 0, 0, 1);
    Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
            0, 0, 1, 0.28, //
            0, 1, 0, 0.13585, //
            -1, 0, 0, 0, //
            0, 0, 0, 1);
    Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
            1, 0, 0, 0, //
            0, 1, 0, -0.1197, //
            0, 0, 1, 0.395, //
            0, 0, 0, 1);
    Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
            1, 0, 0, 0, //
            0, 1, 0, 0, //
            0, 0, 1, 0.14225, //
            0, 0, 0, 1);
    Matrix<N6, N6> G1 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
            0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7));
    Matrix<N6, N6> G2 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
            0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393));
    Matrix<N6, N6> G3 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
            0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275));
    List<Matrix<N6, N6>> Glist = List.of(G1, G2, G3);
    List<Matrix<N4, N4>> Mlist = List.of(M01, M12, M23, M34);
    List<Vector<N6>> Slist = List.of(//
            VecBuilder.fill(1, 0, 1, 0, 1, 0),
            VecBuilder.fill(0, 1, 0, -0.089, 0, 0),
            VecBuilder.fill(0, 1, 0, -0.089, 0, 0.425));

    @Test
    void testad() {
        Matrix<N6, N1> V = MatBuilder.fill(Nat.N6(), Nat.N1(), //
                1, 2, 3, 4, 5, 6);
        Matrix<N6, N6> actual = ModernRobotics.ad(V);
        Matrix<N6, N6> expected = MatBuilder.fill(Nat.N6(), Nat.N6(), //
                0, -3, 2, 0, 0, 0, //
                3, 0, -1, 0, 0, 0, //
                -2, 1, 0, 0, 0, 0, //
                0, -6, 5, 0, -3, 2, //
                6, 0, -4, 3, 0, -1, //
                -5, 4, 0, -2, 1, 0);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testInverseDynamics() {
        Vector<N3> actual = ModernRobotics.InverseDynamics(
                Nat.N3(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist);
        Vector<N3> expected = VecBuilder.fill(74.69616155, -33.06766016, -3.23057314);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testMassMatrix() {
        Matrix<N3, N3> actual = ModernRobotics.MassMatrix(
                Nat.N3(), thetalist, Mlist, Glist, Slist);
        Matrix<N3, N3> expected = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                2.25433380e+01, -3.07146754e-01, -7.18426391e-03, //
                -3.07146754e-01, 1.96850717e+00, 4.32157368e-01, //
                -7.18426391e-03, 4.32157368e-01, 1.91630858e-01);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testVelQuadraticForces() {
        Vector<N3> actual = ModernRobotics.VelQuadraticForces(
                Nat.N3(), thetalist, dthetalist, Mlist, Glist, Slist);
        Vector<N3> expected = VecBuilder.fill(
                0.26453118, -0.05505157, -0.00689132);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testGravityForces() {
        Vector<N3> actual = ModernRobotics.GravityForces(
                Nat.N3(), thetalist, g, Mlist, Glist, Slist);
        Vector<N3> expected = VecBuilder.fill(
                28.40331262, -37.64094817, -5.4415892);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testEndEffectorForces() {
        Vector<N3> actual = ModernRobotics.EndEffectorForces(
                Nat.N3(), thetalist, Ftip, Mlist, Glist, Slist);
        Vector<N3> expected = VecBuilder.fill(1.40954608, 1.85771497, 1.392409);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testForwardDynamics() {
        Vector<N3> actual = ModernRobotics.ForwardDynamics(
                Nat.N3(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist);
        Vector<N3> expected = VecBuilder.fill(-0.97392907, 25.58466784, -32.91499212);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testTransInv() {
        Matrix<N4, N4> T = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 0, -1, 0, //
                0, 1, 0, 3, //
                0, 0, 0, 1);
        Matrix<N4, N4> actual = ModernRobotics.TransInv(T);
        Matrix<N4, N4> expected = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0,
                0, 0, 1, -3, //
                0, -1, 0, 0, //
                0, 0, 0, 1);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testTransToRp() {
        Matrix<N4, N4> T = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 0, -1, 0, //
                0, 1, 0, 3, //
                0, 0, 0, 1);
        Pair<Matrix<N3, N3>, Matrix<N3, N1>> actual = ModernRobotics.TransToRp(T);
        Matrix<N3, N3> R = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                1, 0, 0, //
                0, 0, -1,
                0, 1, 0);
        Matrix<N3, N1> p = MatBuilder.fill(Nat.N3(), Nat.N1(), //
                0, //
                0, //
                3);
        TestUtil.verify(R, actual.getFirst());
        TestUtil.verify(p, actual.getSecond());
    }

    @Test
    void testAdjoint() {
        Matrix<N4, N4> T = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 0, -1, 0, //
                0, 1, 0, 3, //
                0, 0, 0, 1);
        Matrix<N6, N6> actual = ModernRobotics.Adjoint(T);
        Matrix<N6, N6> expected = MatBuilder.fill(Nat.N6(), Nat.N6(),
                1, 0, 0, 0, 0, 0, //
                0, 0, -1, 0, 0, 0, //
                0, 1, 0, 0, 0, 0, //
                0, 0, 3, 1, 0, 0, //
                3, 0, 0, 0, 0, -1, //
                0, 0, 0, 0, 1, 0);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testVecToso3() {
        Vector<N3> omg = VecBuilder.fill(1, 2, 3);
        Matrix<N3, N3> actual = ModernRobotics.VecToso3(omg);
        Matrix<N3, N3> expected = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -3, 2, //
                3, 0, -1, //
                -2, 1, 0);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testMatrixExp6() {
        Matrix<N4, N4> se3mat = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0, 0, 0, 0, //
                0, 0, -1.57079632, 2.35619449, //
                0, 1.57079632, 0, 2.35619449, //
                0, 0, 0, 0);
        Matrix<N4, N4> actual = ModernRobotics.MatrixExp6(se3mat);
        Matrix<N4, N4> expected = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1.0, 0.0, 0.0, 0.0, //
                0.0, 0.0, -1.0, 0.0, //
                0.0, 1.0, 0.0, 3.0, //
                0, 0, 0, 1);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testso3ToVec() {
        Matrix<N3, N3> so3mat = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -3, 2, //
                3, 0, -1, //
                -2, 1, 0);
        Matrix<N3, N1> actual = ModernRobotics.so3ToVec(so3mat);
        Matrix<N3, N1> expected = MatBuilder.fill(Nat.N3(), Nat.N1(), //
                1, 2, 3);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testAxisAng3() {
        Vector<N3> expc3 = VecBuilder.fill(1, 2, 3);
        Pair<Matrix<N3, N1>, Double> aa = ModernRobotics.AxisAng3(expc3);
        Vector<N3> axis = VecBuilder.fill(0.26726124, 0.53452248, 0.80178373);
        double angle = 3.7416573867739413;
        TestUtil.verify(axis, aa.getFirst());
        assertEquals(angle, aa.getSecond(), 1e-6);
    }

    @Test
    void testMatrixExp3() {
        Matrix<N3, N3> so3mat = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -3, 2, //
                3, 0, -1, //
                -2, 1, 0);
        Matrix<N3, N3> actual = ModernRobotics.MatrixExp3(so3mat);
        Matrix<N3, N3> expected = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                -0.69492056, 0.71352099, 0.08929286, //
                -0.19200697, -0.30378504, 0.93319235, //
                0.69297817, 0.6313497, 0.34810748);
        TestUtil.verify(expected, actual);
    }

    @Test
    void testVecTose3() {
        Matrix<N6, N1> V = MatBuilder.fill(Nat.N6(), Nat.N1(), //
                1, 2, 3, 4, 5, 6);
        Matrix<N4, N4> actual = ModernRobotics.VecTose3(V);
        Matrix<N4, N4> expected = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0, -3, 2, 4, //
                3, 0, -1, 5, //
                -2, 1, 0, 6, //
                0, 0, 0, 0);
        TestUtil.verify(expected, actual);
    }

}
