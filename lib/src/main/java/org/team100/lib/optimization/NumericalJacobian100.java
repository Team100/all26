package org.team100.lib.optimization;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Similar to the WPI version but using vectors instead of matrices.
 * 
 * Estimates the Jacobian using symmetric differences around the reference x.
 * 
 * This previously included a one-sided version, but it caused orbiting around
 * the goal, since the derivative is positive to the right and negative to the
 * left (thus zero in the two-sided version, and, wrongly, nonzero in the
 * one-sided version).
 */
public class NumericalJacobian100 {
    // Using too-small a dx means trouble when the function at hand
    // is itself discrete, e.g. the integrated Drag model.
    // See VariableVelocityShootingMethodTest.testJacobian()
    // Note: using a small DX reduces accuracy somewhat
    // This value produced nonsense:
    // private static final double DX = 1e-5;
    // This value seems to work:
    private static final double DX = 1e-3;

    /**
     * Estimates the Jacobian using symmetric differences around the reference x.
     * 
     * Mutates x to save allocation cost, but puts it back the way it was upon
     * returning.
     * 
     * @param <X>  x dimensions
     * @param <Y>  y dimensions
     * @param xdim x dimensions
     * @param ydim y dimensions
     * @param f    function, y=f(x)
     * @param x    x value to evaluate
     * @returns jacobian, e.g.
     *          [dy1/dx1 dy1/dx2]
     *          [dy2/dx1 dy2/dx2]
     */
    public static <Y extends Num, X extends Num> Matrix<Y, X> numericalJacobian(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> x) {
        Matrix<Y, X> result = new Matrix<>(ydim, xdim);
        for (int i = 0; i < xdim.getNum(); i++) {
            Vector<X> dxPlus = new Vector<>(x.getStorage().copy());
            Vector<X> dxMinus = new Vector<>(x.getStorage().copy());
            dxPlus.set(i, 0, dxPlus.get(i, 0) + DX);
            dxMinus.set(i, 0, dxMinus.get(i, 0) - DX);
            Vector<Y> dF = f.apply(dxPlus).minus(f.apply(dxMinus)).div(2 * DX);
            result.setColumn(i, Matrix.changeBoundsUnchecked(dF));
        }
        return result;
    }

    /**
     * 1d specialization of above that avoids vectors
     */
    public static double numericalJacobian1d(DoubleUnaryOperator f, double x) {
        return (f.applyAsDouble(x + DX) - f.applyAsDouble(x - DX)) / (2 * DX);
    }
}
