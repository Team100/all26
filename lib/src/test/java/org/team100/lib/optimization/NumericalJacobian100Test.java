package org.team100.lib.optimization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class NumericalJacobian100Test {
    /** Scalar function, f(x) = x. */
    @Test
    void test1() {
        Nat<N1> xdim = Nat.N1();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N1>, Vector<N1>> f = x -> x;
        Vector<N1> x = VecBuilder.fill(0);
        Matrix<N1, N1> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
    }

    /** Scalar function, f(x) = x. */
    @Test
    void test1_1d() {
        DoubleUnaryOperator f = x -> x;
        double x = 0;
        double j = NumericalJacobian100.numericalJacobian1d(f, x);
        assertEquals(1, j, 1e-9);
    }

    /** Scalar function, f(x) = 2x. */
    @Test
    void test2() {
        Nat<N1> xdim = Nat.N1();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N1>, Vector<N1>> f = x -> x.times(2);
        Vector<N1> x = VecBuilder.fill(1);
        Matrix<N1, N1> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
    }

    /** Scalar function, f(x) = 2x. */
    @Test
    void test2_1d() {
        DoubleUnaryOperator f = x -> 2 * x;
        double x = 1;
        double j = NumericalJacobian100.numericalJacobian1d(f, x);
        assertEquals(2, j, 1e-9);
    }

    /** Multivariate scalar function, f(x) = norm(x)^2 */
    @Test
    void test3() {
        Nat<N2> xdim = Nat.N2();
        Nat<N1> ydim = Nat.N1();
        Function<Vector<N2>, Vector<N1>> f = x -> VecBuilder.fill(Math.pow(x.normF(), 2));
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        Matrix<N1, N2> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
        assertEquals(1, j.get(0, 1), 1e-9);
    }

    /** Multivariate vector function, f(x) = x */
    @Test
    void test4() {
        Nat<N2> xdim = Nat.N2();
        Nat<N2> ydim = Nat.N2();
        Function<Vector<N2>, Vector<N2>> f = x -> x;
        Vector<N2> x = VecBuilder.fill(1, 1);
        Matrix<N2, N2> j = NumericalJacobian100.numericalJacobian(xdim, ydim, f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
        assertEquals(0, j.get(0, 1), 1e-9);
        assertEquals(0, j.get(1, 0), 1e-9);
        assertEquals(1, j.get(1, 1), 1e-9);
        Matrix<N2, N2> jInv = j.inv();
        assertEquals(1, jInv.get(0, 0), 1e-9);
        assertEquals(0, jInv.get(0, 1), 1e-9);
        assertEquals(0, jInv.get(1, 0), 1e-9);
        assertEquals(1, jInv.get(1, 1), 1e-9);
    }

    /** 2d RR arm */
    @Test
    void testRR() {
        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        Vector<N2> Xd = VecBuilder.fill(0, 1);

        Function<Vector<N2>, Vector<N2>> fwd = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        // estimate - goal so that the jacobian is the same.
        Function<Vector<N2>, Vector<N2>> err = q -> fwd.apply(q).minus(Xd);

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Vector<N2> X0 = fwd.apply(q0);
        assertEquals(1, X0.get(0), 1e-9);
        assertEquals(1, X0.get(1), 1e-9);
        // jacobian at q0
        Matrix<N2, N2> j0 = NumericalJacobian100.numericalJacobian(
                Nat.N2(), Nat.N2(), err, q0);
        // radius of rotation for q0 to end, at pi/2, is sqrt(2)
        // dx/dq for unit circle is -sqrt(2)/2
        // dx/dq0
        assertEquals(-1, j0.get(0, 0), 1e-3);
        // q1 is opposite to x with unit radius
        // dx/dq1
        assertEquals(-1, j0.get(0, 1), 1e-5);
        // same as first case but positive
        // dy/dq0
        assertEquals(1, j0.get(1, 0), 1e-3);
        // q1 doesn't change y
        // dy/dq1
        assertEquals(0, j0.get(1, 1), 1e-3);
    }

}
