package org.team100.math;

import java.util.function.BiFunction;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.util.Num;

public class EulerIntegrator<States extends Num, Inputs extends Num> implements Integrator<States, Inputs> {

    @Override
    public Matrix<States, N1> integrate(
            BiFunction<Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1>> f,
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u,
            double dt) {
        Matrix<States, N1> xdot = f.apply(x, u);
        Matrix<States, N1> dx = xdot.times(dt);
        return x.plus(dx);
    }
}