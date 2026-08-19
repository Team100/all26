package org.team100.control;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.util.Num;

/** Always returns the same value. */
public class ConstantControl<States extends Num, Inputs extends Num>
        implements ControlLaw<States, Inputs> {
    private final Matrix<Inputs, N1> u;

    public ConstantControl(Matrix<Inputs, N1> u) {
        this.u = u;
    }

    @Override
    public Matrix<Inputs, N1> f(Matrix<States, N1> x) {
        return u;
    }

}
