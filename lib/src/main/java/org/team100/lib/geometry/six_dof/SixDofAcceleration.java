package org.team100.lib.geometry.six_dof;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N6;

public record SixDofAcceleration(
        double q1ddot,
        double q2ddot,
        double q3ddot,
        double q4ddot,
        double q5ddot,
        double q6ddot) {

    public static SixDofAcceleration zero() {
        return new SixDofAcceleration(0, 0, 0, 0, 0, 0);
    }

    public static SixDofAcceleration fromVector(Matrix<N6, N1> v) {
        return new SixDofAcceleration(
                v.get(0, 0),
                v.get(1, 0),
                v.get(2, 0),
                v.get(3, 0),
                v.get(4, 0),
                v.get(5, 0));
    }

    public Vector<N6> toVector() {
        return VecBuilder.fill(
                q1ddot,
                q2ddot,
                q3ddot,
                q4ddot,
                q5ddot,
                q6ddot);
    }

    public static SixDofAcceleration fromList(List<ControlR1> setpoint) {
        return new SixDofAcceleration(
                setpoint.get(0).a(),
                setpoint.get(1).a(),
                setpoint.get(2).a(),
                setpoint.get(3).a(),
                setpoint.get(4).a(),
                setpoint.get(5).a());
    }
}
