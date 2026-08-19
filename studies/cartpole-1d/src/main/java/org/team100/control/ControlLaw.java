package org.team100.control;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.util.Num;

/**
 * Control input is a function of the error state.
 * 
 * @param States dimensions of error state, x
 * @param Inputs dimensions of control input u
 */
public interface ControlLaw<States extends Num, Inputs extends Num> {
    Matrix<Inputs, N1> f(Matrix<States, N1> x);
}
