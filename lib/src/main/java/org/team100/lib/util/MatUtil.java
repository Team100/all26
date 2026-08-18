package org.team100.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

public class MatUtil {
    /** Return an NxN matrix with the given vector on the diagonal. */
    public static <N extends Num> Matrix<N, N> diag(Nat<N> n, Vector<N> v) {
        if (n.getNum() != v.getNumRows())
            throw new IllegalArgumentException();
        Matrix<N, N> result = Matrix.eye(n);
        for (int i = 0; i < n.getNum(); ++i) {
            result.set(i, i, v.get(i));
        }
        return result;
    }

    /** Throw if the trace is not positive. */
    public static void positiveTrace(Matrix<?, ?> m) {
        if (m.trace() > 0)
            return;
        throw new IllegalArgumentException();
    }
}
