package org.team100.lib.optimization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.function.Function;

import org.ejml.data.SingularMatrixException;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.kinematics.urdf.URDFAL5D;
import org.team100.lib.util.StrUtil;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.util.Nat;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N5;
import org.wpilib.math.numbers.N6;

public class NewtonsMethodTest {
    private static final boolean DEBUG = false;

    /**
     * Multivariate scalar function, f(x) = norm(x)^2.
     * 
     * This is a common scalar "error" or "loss" function, but the normal matrix
     * solver doesn't like it.
     */
    @Test
    void testScalar() {
        Function<Vector<N2>, Vector<N1>> f = x -> VecBuilder.fill(Math.pow(x.normF(), 2));
        Vector<N2> x = VecBuilder.fill(1, 0.5);
        Vector<N1> Y = f.apply(x);
        assertEquals(1.25, Y.get(0), 1e-9);

        Matrix<N1, N2> j = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N1(), f, x);
        assertEquals(2, j.get(0, 0), 1e-9);
        assertEquals(1, j.get(0, 1), 1e-9);

        {
            // pseudoinverse
            Matrix<N2, N1> jInv = new Matrix<>(j.getStorage().pseudoInverse());
            assertEquals(0.4, jInv.get(0, 0), 1e-9);
            assertEquals(0.2, jInv.get(1, 0), 1e-9);
            Vector<N2> dx = new Vector<>(jInv.times(Y));
            Vector<N2> x2 = x.minus(dx);
            assertEquals(0.5, x2.get(0), 1e-9);
            assertEquals(0.25, x2.get(1), 1e-9);
        }
        {
            // solve (does not work)
            // Vector<N2> dx = new Vector<>(j.solve(Y.times(-1)));
            // Vector<N2> x2 = x.plus(dx);
            // assertEquals(-100, x2.get(0), 1e-9);
            // assertEquals(-100, x2.get(1), 1e-9);
        }
    }

    /** Multivariate vector function, f(x) = x */
    @Test
    void test2() {
        Function<Vector<N2>, Vector<N2>> f = x -> x;
        Vector<N2> x = VecBuilder.fill(1, 1);
        Vector<N2> Y = f.apply(x);
        assertEquals(1, Y.get(0), 1e-9);
        assertEquals(1, Y.get(1), 1e-9);

        // jacobian is identity
        Matrix<N2, N2> j = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N2(), f, x);
        assertEquals(1, j.get(0, 0), 1e-9);
        assertEquals(0, j.get(0, 1), 1e-9);
        assertEquals(0, j.get(1, 0), 1e-9);
        assertEquals(1, j.get(1, 1), 1e-9);
        // in this case it's invertible; inverse is identity.
        Matrix<N2, N2> jInverse = j.inv();
        assertEquals(1, jInverse.get(0, 0), 1e-9);
        assertEquals(0, jInverse.get(0, 1), 1e-9);
        assertEquals(0, jInverse.get(1, 0), 1e-9);
        assertEquals(1, jInverse.get(1, 1), 1e-9);
        {
            // pseudoinverse
            Matrix<N2, N2> jInv = new Matrix<>(j.getStorage().pseudoInverse());
            assertEquals(1, jInv.get(0, 0), 1e-9);
            assertEquals(0, jInv.get(0, 1), 1e-9);
            assertEquals(0, jInv.get(1, 0), 1e-9);
            assertEquals(1, jInv.get(1, 1), 1e-9);

            // since f is linear the Newton method works perfectly:
            Vector<N2> dx = new Vector<>(jInv.times(Y));
            Vector<N2> x2 = x.minus(dx);
            assertEquals(0, x2.get(0), 1e-9);
            assertEquals(0, x2.get(1), 1e-9);
        }
        {
            // solve also works
            Vector<N2> dx = new Vector<>(j.solve(Y.times(-1)));
            Vector<N2> x2 = x.plus(dx);
            assertEquals(0, x2.get(0), 1e-9);
            assertEquals(0, x2.get(1), 1e-9);
        }
    }

    @Test
    void test3() {
        // 2d RR arm example, f translation as a function of joint angles q

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        Vector<N2> Xd = VecBuilder.fill(0, 1);

        Function<Vector<N2>, Vector<N2>> fwd = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        // this will be fwd in the goal frame
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
        if (DEBUG)
            System.out.println(StrUtil.matStr(j0));
        // dx/dq0
        assertEquals(-1, j0.get(0, 0), 1e-6);
        // dx/dq1
        assertEquals(-1, j0.get(0, 1), 1e-6);
        // dy/dq0
        assertEquals(1, j0.get(1, 0), 1e-6);
        // dy/dq1
        assertEquals(0, j0.get(1, 1), 1e-6);

        // initial error
        Vector<N2> err0 = err.apply(q0);
        // printxy(err0);
        assertEquals(1, err0.get(0), 1e-9);
        assertEquals(0, err0.get(1), 1e-9);

        Vector<N2> dq1 = new Vector<>(j0.solve(err0));
        // matches my hand-guess
        // q0 (0,1.5), dq(0,-1), so q1 (0,2.5)
        assertEquals(0, dq1.get(0), 1e-9);
        assertEquals(-1, dq1.get(1), 1e-3);

        Vector<N2> q1 = q0.minus(dq1);
        assertEquals(0, q1.get(0), 1e-9);
        assertEquals(2.571, q1.get(1), 1e-3);

        Matrix<N2, N2> j1 = NumericalJacobian100.numericalJacobian(
                Nat.N2(), Nat.N2(), err, q1);
        // since q1 has overshot and q0 hasn't moved, x is still a little over and y is
        // under
        Vector<N2> err1 = err.apply(q1);
        // printxy(err1);
        assertEquals(0.158, err1.get(0), 1e-3);
        assertEquals(-0.460, err1.get(1), 1e-3);

        Vector<N2> dq2 = new Vector<>(j1.solve(err1));
        assertEquals(-0.706, dq2.get(0), 1e-3);
        assertEquals(0.413, dq2.get(1), 1e-3);

        Vector<N2> q2 = q1.minus(dq2);
        assertEquals(0.707, q2.get(0), 1e-3);
        assertEquals(2.158, q2.get(1), 1e-3);

        Matrix<N2, N2> j2 = NumericalJacobian100.numericalJacobian(
                Nat.N2(), Nat.N2(), err, q2);

        // now x and y have overshot
        Vector<N2> err2 = err.apply(q2);
        // printxy(err2);
        assertEquals(-0.201, err2.get(0), 1e-3);
        assertEquals(-0.077, err2.get(1), 1e-3);

        Vector<N2> dq3 = new Vector<>(j2.solve(err2));
        assertEquals(0.207, dq3.get(0), 1e-3);
        assertEquals(0.037, dq3.get(1), 1e-3);

        Vector<N2> q3 = q2.minus(dq3);
        assertEquals(0.5, q3.get(0), 1e-3);
        assertEquals(2.121, q3.get(1), 1e-3);

        Matrix<N2, N2> j3 = NumericalJacobian100.numericalJacobian(
                Nat.N2(), Nat.N2(), err, q3);
        // Vector<N2> X3 = fwd.apply(q3);
        Vector<N2> err3 = err.apply(q3);
        // printxy(err3);
        assertEquals(0.011, err3.get(0), 1e-3);
        assertEquals(-0.023, err3.get(1), 1e-3);

        Vector<N2> dq4 = new Vector<>(j3.solve(err3));
        assertEquals(-0.024, dq4.get(0), 1e-3);
        assertEquals(0.026, dq4.get(1), 1e-3);

        Vector<N2> q4 = q3.minus(dq4);
        assertEquals(0.524, q4.get(0), 1e-3);
        assertEquals(2.094, q4.get(1), 1e-3);
    }

    void printxy(Vector<N2> err) {
        if (DEBUG)
            System.out.println(StrUtil.vecStr(err));
    }

    @Test
    void test4() {
        // Same as above but including end angle. note that end angle
        // can't be specified independently of position. so what happens?
        // when we specify the right angle, it speeds up the solver a bit.

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6
        Vector<N3> Xd = VecBuilder.fill(0, 1, 2.618);

        Function<Vector<N2>, Vector<N3>> fwd = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                q.get(0) + q.get(1));
        Function<Vector<N2>, Vector<N3>> err = q -> fwd.apply(q).minus(Xd);

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Vector<N3> X0 = fwd.apply(q0);
        assertEquals(1, X0.get(0), 1e-9);
        assertEquals(1, X0.get(1), 1e-9);

        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q0);
        if (DEBUG)
            System.out.println(StrUtil.matStr(j0));
        // dx/dq0
        assertEquals(-1, j0.get(0, 0), 1e-6);
        // dx/dq1
        assertEquals(-1, j0.get(0, 1), 1e-6);
        // dy/dq0
        assertEquals(1, j0.get(1, 0), 1e-6);
        // dy/dq1
        assertEquals(0, j0.get(1, 1), 1e-6);
        // angles just add
        // dt/dq0
        assertEquals(1, j0.get(2, 0), 1e-6);
        // dt/dq1
        assertEquals(1, j0.get(2, 1), 1e-6);

        Vector<N3> err0 = err.apply(q0);
        Vector<N2> dq1 = new Vector<>(j0.solve(err0));

        Vector<N2> q1 = q0.minus(dq1);
        assertEquals(0, q1.get(0), 1e-9);
        assertEquals(2.594, q1.get(1), 1e-3);

        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q1);
        Vector<N3> err1 = err.apply(q1);

        Vector<N2> dq2 = new Vector<>(j1.solve(err1));
        Vector<N2> q2 = q1.minus(dq2);
        assertEquals(0.547, q2.get(0), 1e-3);
        assertEquals(2.126, q2.get(1), 1e-3);

        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q2);
        Vector<N3> err2 = err.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(err2));
        Vector<N2> q3 = q2.minus(dq3);
        assertEquals(0.522, q3.get(0), 1e-3);
        assertEquals(2.096, q3.get(1), 1e-3);

        // by step 3 we have the answer
        Matrix<N3, N2> j3 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q3);
        Vector<N3> err3 = err.apply(q3);
        Vector<N2> dq4 = new Vector<>(j3.solve(err3));
        Vector<N2> q4 = q3.minus(dq4);
        assertEquals(0.524, q4.get(0), 1e-3);
        assertEquals(2.094, q4.get(1), 1e-3);
    }

    @Test
    void test4Pose2() {
        // Same as above but using pose2d. This does what GTSAM does, which is to
        // optimize in the tangent space (pose->log->twist->vector) .

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6
        Pose2d XXd = new Pose2d(0, 1, new Rotation2d(2.618));

        Function<Vector<N2>, Pose2d> fwd = q -> new Pose2d(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                new Rotation2d(q.get(0) + q.get(1)));

        Function<Vector<N2>, Vector<N3>> err = q -> GeometryUtil.toVec(fwd.apply(q).minus(XXd).log());

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Pose2d XX0 = fwd.apply(q0);
        assertEquals(1, XX0.getX(), 1e-9);
        assertEquals(1, XX0.getY(), 1e-9);
        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q0);
        if (DEBUG)
            System.out.println(StrUtil.matStr(j0));

        // note the jacobian is different since the "log" is in there and we are far
        // from the goal.
        assertEquals(1.024, j0.get(0, 0), 1e-3);
        assertEquals(0.117, j0.get(0, 1), 1e-3);
        assertEquals(0.726, j0.get(1, 0), 1e-3);
        assertEquals(1.249, j0.get(1, 1), 1e-3);
        assertEquals(1, j0.get(2, 0), 1e-3);
        assertEquals(1, j0.get(2, 1), 1e-3);

        Vector<N3> err0 = err.apply(q0);
        Vector<N2> dq1 = new Vector<>(j0.solve(err0));

        Vector<N2> q1 = q0.minus(dq1);
        assertEquals(0.487, q1.get(0), 1e-3);
        assertEquals(2.058, q1.get(1), 1e-3);

        // including the pose makes it converge faster
        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q1);
        Vector<N3> err1 = err.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(err1));
        Vector<N2> q2 = q1.minus(dq2);
        assertEquals(0.524, q2.get(0), 1e-3);
        assertEquals(2.095, q2.get(1), 1e-3);

        // there in step 2
        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q2);
        Vector<N3> err2 = err.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(err2));
        Vector<N2> q3 = q2.minus(dq3);
        assertEquals(0.524, q3.get(0), 1e-3);
        assertEquals(2.094, q3.get(1), 1e-3);
    }

    @Test
    void testLinear1d() {
        // linear function: f(x) = x + 1
        // exact answer in one iteration
        // 2 microseconds on my desktop machine.
        Function<Vector<N1>, Vector<N1>> f = //
                x -> x.plus(VecBuilder.fill(1));
        Vector<N1> q0 = VecBuilder.fill(0);
        Vector<N1> minQ = VecBuilder.fill(-10);
        Vector<N1> maxQ = VecBuilder.fill(10);
        NewtonsMethod<N1, N1> s = new NewtonsMethod<>(
                Nat.N1(), Nat.N1(), f, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1));

        // f(-1) = -1 + 1 = 0
        Vector<N1> x = s.solve(q0, 1, true);
        assertEquals(-1, x.get(0), 1e-3);
    }

    @Test
    void testLinear1d2() {
        // linear function: f(x) = x + 1
        // goal = 2
        // error function: err(x) = 2 - f(x)
        // f(x) = 2
        // x+1 = 2
        // x = 1
        Function<Vector<N1>, Vector<N1>> f = //
                x -> x.plus(VecBuilder.fill(1));
        Vector<N1> goal = VecBuilder.fill(2);
        Function<Vector<N1>, Vector<N1>> err = //
                x -> goal.minus(f.apply(x));
        Vector<N1> q0 = VecBuilder.fill(0);
        Vector<N1> minQ = VecBuilder.fill(-10);
        Vector<N1> maxQ = VecBuilder.fill(10);
        NewtonsMethod<N1, N1> s = new NewtonsMethod<>(
                Nat.N1(), Nat.N1(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1));

        Vector<N1> x = s.solve(q0, 1, true);
        assertEquals(1, x.get(0), 1e-3);
    }

    @Test
    void testLinear1dPerformance() {
        // 2 microseconds on my desktop machine.
        Function<Vector<N1>, Vector<N1>> f = x -> x.plus(VecBuilder.fill(1));
        Vector<N1> q0 = VecBuilder.fill(0);
        Vector<N1> minQ = VecBuilder.fill(-10);
        Vector<N1> maxQ = VecBuilder.fill(10);
        NewtonsMethod<N1, N1> s = new NewtonsMethod<>(
                Nat.N1(), Nat.N1(), f, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1));
        long startTime = System.nanoTime();
        int iter = 0;
        int maxIter = 100000;
        for (iter = 0; iter < maxIter; ++iter) {
            s.solve(q0, 1, true);
        }
        long finishTime = System.nanoTime();
        if (DEBUG) {
            double et = ((double) finishTime - startTime) / 1000000;
            System.out.printf("iterations: %d ET (ms): %6.3f ET per iter (ms) %6.3f\n",
                    iter, et, et / maxIter);
        }
    }

    // this fails sometimes
    // @Test
    void testQuadratic1d() {
        // quadratic function: f(x) = x^2 - 2
        // good answer in 4 iterations
        Function<Vector<N1>, Vector<N1>> f = x -> VecBuilder.fill(x.get(0) * x.get(0) - 2);
        Vector<N1> q0 = VecBuilder.fill(0);
        Vector<N1> minQ = VecBuilder.fill(-10);
        Vector<N1> maxQ = VecBuilder.fill(10);
        NewtonsMethod<N1, N1> s = new NewtonsMethod<>(
                Nat.N1(), Nat.N1(), f, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1));
        Vector<N1> x = s.solve(q0, 1, true);
        // f(1.414) = 2 - 2 = 0
        // note +/-
        assertEquals(1.414, Math.abs(x.get(0)), 1e-3);
    }

    @Test
    void test4Pose2Solver() {
        // case above but using the solver
        Pose2d XXd = new Pose2d(0, 1, new Rotation2d(2.618));
        Function<Vector<N2>, Pose2d> fwd = q -> new Pose2d(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                new Rotation2d(q.get(0) + q.get(1)));
        Function<Vector<N2>, Vector<N3>> err = q -> GeometryUtil.toVec(fwd.apply(q).minus(XXd).log());
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N3> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N3(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        Vector<N2> x = s.solve(q0, 0, false);
        assertEquals(0.524, x.get(0), 1e-3);
        assertEquals(2.094, x.get(1), 1e-3);
    }

    @Test
    void testFailedSolve() {
        Vector<N2> b = VecBuilder.fill(0, 0);
        Matrix<N2, N2> A = new Matrix<>(Nat.N2(), Nat.N2());
        assertThrows(SingularMatrixException.class, () -> A.solve(b));
    }

    @Test
    void test4Pose2Solver2() {
        // case above but using the solver
        Pose2d XXd = new Pose2d(0, 1, new Rotation2d(2.618));
        Function<Vector<N2>, Pose2d> fwd = q -> new Pose2d(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                new Rotation2d(q.get(0) + q.get(1)));
        Function<Vector<N2>, Vector<N3>> err = q -> GeometryUtil.toVec(fwd.apply(q).minus(XXd).log());
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N3> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N3(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        Vector<N2> x = s.solve(q0, 5, true);
        assertEquals(0.524, x.get(0), 1e-3);
        assertEquals(2.094, x.get(1), 1e-3);
    }

    @Test
    void test4Pose2Transform2() {
        // Same as above but using pose2d and transform2d for the forward f, so that
        // it's more like an arbitrary kinematic chain. Same convergence as above.
        // link lengths
        final double l0 = 1;
        final double l1 = 1;

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6
        Pose2d XXd = new Pose2d(0, 1, new Rotation2d(2.618));

        Function<Vector<N2>, Pose2d> fwd = q -> Pose2d.kZero
                .transformBy(new Transform2d(0, 0, new Rotation2d(q.get(0))))
                .transformBy(new Transform2d(l0, 0, Rotation2d.kZero))
                .transformBy(new Transform2d(0, 0, new Rotation2d(q.get(1))))
                .transformBy(new Transform2d(l1, 0, Rotation2d.kZero));
        Function<Vector<N2>, Vector<N3>> err = q -> GeometryUtil.toVec(fwd.apply(q).minus(XXd).log());

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // initial position
        Pose2d XX0 = fwd.apply(q0);
        assertEquals(1, XX0.getX(), 1e-9);
        assertEquals(1, XX0.getY(), 1e-9);

        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q0);
        if (DEBUG)
            System.out.println(StrUtil.matStr(j0));

        // note the jacobian is different since the "log" is in there.
        assertEquals(1.024, j0.get(0, 0), 1e-3);
        assertEquals(0.117, j0.get(0, 1), 1e-3);
        assertEquals(0.726, j0.get(1, 0), 1e-3);
        assertEquals(1.249, j0.get(1, 1), 1e-3);
        assertEquals(1, j0.get(2, 0), 1e-3);
        assertEquals(1, j0.get(2, 1), 1e-3);

        Vector<N3> err0 = err.apply(q0);
        Vector<N2> dq1 = new Vector<>(j0.solve(err0));
        Vector<N2> q1 = q0.minus(dq1);
        assertEquals(0.487, q1.get(0), 1e-3);
        assertEquals(2.058, q1.get(1), 1e-3);

        // this is a bit better convergence than the case above
        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q1);
        Vector<N3> err1 = err.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(err1));
        Vector<N2> q2 = q1.minus(dq2);
        assertEquals(0.524, q2.get(0), 1e-3);
        assertEquals(2.095, q2.get(1), 1e-3);

        // there in step 2
        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q2);
        Vector<N3> err2 = err.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(err2));
        Vector<N2> q3 = q2.minus(dq3);
        assertEquals(0.524, q3.get(0), 1e-3);
        assertEquals(2.094, q3.get(1), 1e-3);
    }

    @Test
    void test5() {
        // Same as above but with an inconsistent end angle: the solver compromises
        // since it's a least-squares thing

        // desired position
        // q0 should be pi/6 or 0.524, q1 should be 2pi/3 or 2.094
        // the resulting end-angle is 5pi/6, so this is wrong.
        Pose2d XXd = new Pose2d(0, 1, new Rotation2d(2));

        Function<Vector<N2>, Pose2d> fwd = q -> new Pose2d(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)),
                new Rotation2d(q.get(0) + q.get(1)));

        Function<Vector<N2>, Vector<N3>> err = q -> GeometryUtil.toVec(fwd.apply(q).minus(XXd).log());

        // initial joint angles
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);

        // jacobian at q0
        Matrix<N3, N2> j0 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q0);
        if (DEBUG)
            System.out.println(StrUtil.matStr(j0));
        assertEquals(0.715, j0.get(0, 0), 1e-3);
        assertEquals(-0.270, j0.get(0, 1), 1e-3);
        assertEquals(0.913, j0.get(1, 0), 1e-3);
        assertEquals(1.127, j0.get(1, 1), 1e-3);
        assertEquals(1, j0.get(2, 0), 1e-9);
        assertEquals(1, j0.get(2, 1), 1e-9);

        Vector<N3> err0 = err.apply(q0);
        Vector<N2> dq1 = new Vector<>(j0.solve(err0));

        Vector<N2> q1 = q0.minus(dq1);
        assertEquals(0.354, q1.get(0), 1e-3);
        assertEquals(1.925, q1.get(1), 1e-3);

        Matrix<N3, N2> j1 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q1);
        Vector<N3> err1 = err.apply(q1);
        Vector<N2> dq2 = new Vector<>(j1.solve(err1));
        Vector<N2> q2 = q1.minus(dq2);
        assertEquals(0.356, q2.get(0), 1e-3);
        assertEquals(1.927, q2.get(1), 1e-3);

        // so this is the compromise
        Matrix<N3, N2> j2 = NumericalJacobian100.numericalJacobian(Nat.N2(), Nat.N3(), err, q2);
        Vector<N3> err2 = err.apply(q2);
        Vector<N2> dq3 = new Vector<>(j2.solve(err2));
        Vector<N2> q3 = q2.minus(dq3);
        assertEquals(0.356, q3.get(0), 1e-3);
        assertEquals(1.927, q3.get(1), 1e-3);

    }

    @Test
    void test6() {
        // case from test3 but using the solver class
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);

        Function<Vector<N2>, Vector<N2>> fwd = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        // this will be fwd in the goal frame
        Function<Vector<N2>, Vector<N2>> err = q -> fwd.apply(q).minus(Xd);

        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N2(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        Vector<N2> x = s.solve(q0, 0, false);
        assertEquals(0.524, x.get(0), 1e-3);
        assertEquals(2.094, x.get(1), 1e-3);
    }

    /** 4 us per solve */
    // disable to speed up tests
    // @Test
    void test7() {
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // performance
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        Function<Vector<N2>, Vector<N2>> err = q -> f.apply(q).minus(Xd);

        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N2(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        int iterations = 1000000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(q0, 0, false);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.println("Newton's solve for RR arm");
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }

    @Test
    void test62() {
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // case from test3 but using the solver class
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        Function<Vector<N2>, Vector<N2>> err = q -> f.apply(q).minus(Xd);

        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N2(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        Vector<N2> x = s.solve(q0, 5, true);
        assertEquals(0.524, x.get(0), 1e-3);
        assertEquals(2.094, x.get(1), 1e-3);
    }

    /** 2.9 us per solve with "solve2" optimizations. */
    @Test
    void test72() {
        Vector<N2> Xd = VecBuilder.fill(0, 1);
        Vector<N2> q0 = VecBuilder.fill(0, Math.PI / 2);
        // performance
        Function<Vector<N2>, Vector<N2>> f = q -> VecBuilder.fill(
                Math.cos(q.get(0)) + Math.cos(q.get(0) + q.get(1)),
                Math.sin(q.get(0)) + Math.sin(q.get(0) + q.get(1)));
        Function<Vector<N2>, Vector<N2>> err = q -> f.apply(q).minus(Xd);

        Vector<N2> minQ = VecBuilder.fill(-Math.PI, -Math.PI);
        Vector<N2> maxQ = VecBuilder.fill(Math.PI, Math.PI);
        NewtonsMethod<N2, N2> s = new NewtonsMethod<>(
                Nat.N2(), Nat.N2(), err, minQ, maxQ, 1e-3, 10, VecBuilder.fill(1, 1));
        // int iterations = 1000000;
        int iterations = 50000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            s.solve(q0, 5, true);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.println("Newton's solve2 for RR arm");
            System.out.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            System.out.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }

    @Test
    void testConvergence() {
        // this is a case from the simulator that doesn't converge.
        URDFAL5D m = URDFAL5D.make();

        Pose3d goal = new Pose3d(
                new Translation3d(0.19991979, 0.0011040928, 0.19832649),
                new Rotation3d(3.3019369e-18, 0.79406969, 7.6530612e-19));
        Function<Vector<N5>, Pose3d> fwd = q -> {
            Pose3d pose = m.forward(m.qMap(q)).get("center_point");
            // System.out.printf("fwd() q %s pose %s\n", StrUtil.vecStr(q),
            // StrUtil.poseStr(pose));
            return pose;
        };
        Function<Vector<N5>, Vector<N6>> err = q -> {
            Pose3d estimate = fwd.apply(q);
            // System.out.printf("estimate %s\n", StrUtil.poseStr(estimate));
            Twist3d twist = estimate.minus(goal).log();
            // System.out.printf("twist %s\n", StrUtil.twistStr(twist));
            return GeometryUtil.toVec(twist);
        };

        double tolerance = 1e-3;
        int iterations = 5;
        Vector<N5> dqLimit = VecBuilder.fill(1, 1, 1, 1, 1);
        int restarts = 10;

        NewtonsMethod<N5, N6> solver = new NewtonsMethod<>(
                Nat.N5(), Nat.N6(), err,
                m.minQ(Nat.N5()), m.maxQ(Nat.N5()),
                tolerance, iterations, dqLimit);

        LynxArmConfig c = new LynxArmConfig(
                2.0994465067e-04,
                -1.8609471376e+00,
                1.5635203893e+00,
                1.0872555301e+00,
                1.4888289270e-04);
        Vector<N5> q0 = c.toVec();
        long startTime = System.nanoTime();
        assertThrows(IllegalArgumentException.class,
                () -> solver.solve(q0, restarts, true));
        if (DEBUG) {
            long finishTime = System.nanoTime();
            System.out.printf("ET (ms): %6.3f\n",
                    ((double) finishTime - startTime) / 1000000);

        }
    }
}
