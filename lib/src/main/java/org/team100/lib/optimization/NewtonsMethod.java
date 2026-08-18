package org.team100.lib.optimization;

import java.util.Random;
import java.util.function.Function;

import org.ejml.data.SingularMatrixException;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.jni.EigenJNI;

/**
 * Newton's method finds a zero of a multivariate function.
 * 
 * For example, supply a function that describes the error in estimate and goal:
 * driving it to zero yields the x values to get the desired f.
 * 
 * Uses the (estimated) Jacobian of the function to estimate the x intercept.
 * 
 * https://en.wikipedia.org/wiki/Newton%27s_method
 * https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
 */
@SuppressWarnings("unused")
public class NewtonsMethod<X extends Num, Y extends Num> {
    private static final int DEBUG = 0;
    private final Nat<X> m_xdim;
    private final Nat<Y> m_ydim;
    private final Function<Vector<X>, Vector<Y>> m_f;
    private final Vector<X> m_xMin;
    private final Vector<X> m_xMax;
    private final double m_tolerance;
    private final int m_iterations;
    /**
     * Max change in estimate per iteration, to avoid overreacting.
     */
    private final Vector<X> m_dxLimit;

    private final Random random = new Random();

    /**
     * 
     * @param xdim       domain dimension
     * @param ydim       codomain dimension
     * @param f          The f function should take euclidean (i.e. independent)
     *                   input and return an error estimate in the tangent manifold
     *                   of whatever space it's actually acting in (e.g. SE(3))
     * @param xMin       minimum x
     * @param xMax       maximum x
     * @param tolerance  return when solution x yields f(x) this close to zero
     * @param iterations when Newton gets stuck, it only takes a few iterations, so
     *                   this doesn't need to be large. Specify a generous
     *                   random-restart limit instead of a large iteration limit.
     * @param dxLimit    maximum step size
     */
    public NewtonsMethod(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> xMin,
            Vector<X> xMax,
            double tolerance,
            int iterations,
            Vector<X> dxLimit) {
        m_xdim = xdim;
        m_ydim = ydim;
        m_f = f;
        m_xMin = xMin;
        m_xMax = xMax;
        m_tolerance = tolerance;
        m_iterations = iterations;
        m_dxLimit = dxLimit;
    }

    /**
     * Find a zero of f.
     * 
     * @param initialX       start here
     * @param restarts       number of random restarts in case of non-convergence
     * @param throwOnFailure throw an exception if we fail to find a solution. Some
     *                       clients can't tolerate a "kinda close" solution.
     */
    public Vector<X> solve(Vector<X> initialX, int restarts, boolean throwOnFailure) {
        // make sure our guess is within the limits.
        limit(initialX);
        if (DEBUG > 1)
            System.out.printf("NewtonsMethod.solve2()\ninitialX: %s\n",
                    StrUtil.vecStr(initialX));
        long startTime = System.nanoTime();
        int iter = 0;
        Vector<Y> error = new Vector<>(m_ydim);
        try {
            // x is the solution estimate
            Vector<X> x = new Vector<>(initialX.getStorage().copy());
            for (iter = 0; iter < m_iterations; ++iter) {

                error = m_f.apply(x);
                if (DEBUG > 0)
                    System.out.printf("iter %d x %s err %s\n",
                            iter, StrUtil.vecStr(x), StrUtil.vecStr(error));

                if (DEBUG > 1)
                    System.out.printf("error: %s\n", StrUtil.vecStr(error));

                if (within(error)) {
                    if (DEBUG > 1)
                        System.out.printf("success iter=%d\n", iter);
                    return x;
                }

                if (!solveOnce(error, x)) {
                    if (DEBUG > 1)
                        System.out.println("solve failed");
                    break;
                }
            }
            if (restarts > 0) {
                if (DEBUG > 1)
                    System.out.println("convergence failed, trying random restart");
                // nearbyStart(x);
                randomStart(x);
                limit(x);
                return solve(x, restarts - 1, throwOnFailure);
            }
            if (DEBUG > 1)
                System.out.printf("random restart failed, error %f\n", error.maxAbs());
            if (throwOnFailure)
                throw new IllegalArgumentException(
                        String.format("failed to converge with error %f for inputs %s",
                                error.maxAbs(),
                                StrUtil.vecStr(initialX)));
            return x;
        } finally {
            long finishTime = System.nanoTime();
            if (DEBUG > 1) {
                System.out.printf("solve2 iterations: %d ET (ms): %6.3f\n", iter,
                        ((double) finishTime - startTime) / 1000000);
            }
        }
    }

    /**
     * This picks a point *near* the failed point, which is probably not what we
     * want, since we'll just fall into the same local minimum.
     */
    void nearbyStart(Vector<X> x) {
        for (int i = 0; i < m_xdim.getNum(); i++) {
            x.set(i, 0, x.get(i) + 0.1 * (random.nextDouble() - 0.5));
        }
    }

    /** Pick a random point within the bounds. */
    private void randomStart(Vector<X> x) {
        for (int i = 0; i < m_xdim.getNum(); i++) {
            double range = m_xMax.get(i) - m_xMin.get(i);
            x.set(i, 0, m_xMin.get(i) + range * random.nextDouble());
        }
    }

    /**
     * @return false if unsolvable
     */
    private boolean solveOnce(Vector<Y> error, Vector<X> x) {
        // Used to use the single-sided jacobian, but that's wrong when the error is at
        // the minimum -- the slope is obviously nonzero on both sides, and opposite, so
        // if you only look at one side, you'll think it is nonzero, and orbit the
        // solution. :-(
        // Note this is the Jacobian of the error, so it's the opposite
        // of the Jacobian of the forward function.
        Matrix<Y, X> J = NumericalJacobian100.numericalJacobian(
                m_xdim, m_ydim, m_f, x);
        if (DEBUG > 1) {
            System.out.printf("x %s\n", StrUtil.vecStr(x));
            System.out.printf("J %s\n", StrUtil.matStr(J));
        }
        try {
            // solve J dx = error
            // see NewtonsMethodTest.testScalar.
            Vector<X> dx;
            if (m_ydim.getNum() >= m_xdim.getNum()) {
                // For "narrow" systems, the normal solver seems to work:
                // dx = new Vector<>(J.solve(error));
                // Bah. The solver is confused by singularities
                // so always use the pseudoinverse.
                Matrix<X, Y> jInv = new Matrix<>(J.getStorage().pseudoInverse());
                dx = new Vector<>(jInv.times(error));
                if (DEBUG > 1)
                    System.out.printf("Jinv %s\n", StrUtil.matStr(jInv));

            } else {
                // QR decomposition also works:
                // Vector<X> dx = getDxWithQRDecomp(error, J);
                // The pseudoinverse should always work (but slower)
                Matrix<X, Y> jInv = new Matrix<>(J.getStorage().pseudoInverse());
                dx = new Vector<>(jInv.times(error));
            }

            if (DEBUG > 1)
                System.out.printf("dx: %s\n", StrUtil.vecStr(dx));

            // Too-high dx results in oscillation.
            clamp(dx);
            update(x, dx);
            // Keep the x estimate within bounds.
            limit(x);
            return true;
        } catch (SingularMatrixException ex) {
            if (DEBUG > 1)
                System.out.println("solver cannot succeed");
            return false;
        }
    }

    /**
     * A solver that allows "wide" undetermined systems, but sometimes returns zero
     * as an answer, which is clearly wrong.
     * 
     * Solves J dx = error for dx
     */
    Vector<X> getDxWithQRDecomp(Vector<Y> error, Matrix<Y, X> J) {
        double[] dst = new double[m_xdim.getNum()];
        EigenJNI.solveFullPivHouseholderQr(
                J.getData(),
                m_ydim.getNum(),
                m_xdim.getNum(),
                error.getData(),
                m_ydim.getNum(),
                1,
                dst);
        return new Vector<>(new Matrix<>(m_xdim, Nat.N1(), dst));
    }

    /**
     * Radial distance is within tolerance.
     */
    private boolean within(Vector<Y> error) {
        return error.norm() < m_tolerance;
    }

    /**
     * Update: x = x - dx.
     * Note the minus sign: negative slope means x should move to the right.
     * Mutates x to save allocations.
     * The "x" space is Euclidean, so using a simple sum is ok.
     */
    private void update(Vector<X> x, Vector<X> dx) {
        if (DEBUG > 1) {
            System.out.println("NewtonsMethod.update()");
            System.out.printf("x %s \n", StrUtil.vecStr(x));
            System.out.printf("dx %s\n", StrUtil.vecStr(dx));
        }
        for (int i = 0; i < x.getNumRows(); ++i) {
            double newXi = x.get(i) - dx.get(i);
            x.set(i, 0, newXi);
        }
        if (DEBUG > 1) {
            System.out.printf("new x %s \n", StrUtil.vecStr(x));
        }
    }

    /**
     * Clamp abs(dx) using a fixed limit.
     * Too-high dx results in oscillation.
     * Mutates dx to save allocations.
     */
    private void clamp(Vector<X> dx) {
        for (int i = 0; i < dx.getNumRows(); ++i) {
            double dxI = dx.get(i);
            double dxLim = m_dxLimit.get(i);
            if (Math.abs(dxI) > dxLim) {
                if (DEBUG > 1)
                    System.out.printf("clamp %d\n", i);
            }
            double clampedDxI = MathUtil.clamp(dxI, -dxLim, dxLim);
            // System.out.printf("clamp %d %15.10f %15.10f\n", i, dxI, clampedDxI);
            dx.set(i, 0, clampedDxI);
        }
    }

    /**
     * Limit x to the per-dimension limits.
     * Mutates x to save allocations.
     */
    private void limit(Vector<X> x) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double xi = x.get(i);
            double xMin = m_xMin.get(i);
            double xMax = m_xMax.get(i);
            xi = MathUtil.clamp(xi, xMin, xMax);
            x.set(i, 0, xi);
        }
    }
}
