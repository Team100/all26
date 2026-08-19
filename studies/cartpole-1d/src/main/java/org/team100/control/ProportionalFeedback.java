package org.team100.control;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.util.Num;

/**
 * Implements u = -Kx.
 */
public class ProportionalFeedback<States extends Num, Inputs extends Num>
        implements ControlLaw<States, Inputs> {
    private final Matrix<Inputs, States> K;

    public ProportionalFeedback(Matrix<Inputs, States> K) {
        this.K = K;
    }

    @Override
    public Matrix<Inputs, N1> f(Matrix<States, N1> x) {
        return K.times(x).times(-1);
    }

}
