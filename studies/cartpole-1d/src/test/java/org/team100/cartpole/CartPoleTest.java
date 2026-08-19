package org.team100.cartpole;

import java.util.List;

import org.jfree.data.xy.XYDataset;
import org.junit.jupiter.api.Test;
import org.team100.control.ConstantControl;
import org.team100.control.ControlLaw;
import org.team100.control.ProportionalFeedback;
import org.team100.lib.util.ChartUtil;
import org.team100.math.Dynamics;
import org.team100.math.Integrator;
import org.team100.math.RK4Integrator;
import org.team100.math.Simulator;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N4;

/** Turn on ChartUtil.SHOW to see anything. */
public class CartPoleTest {
    private static final double LENGTH = 1.0;
    private static final double DT = 0.01;
    private static final double ET = 15.0;
    private static final int SCALE = 500;

    @Test
    void testSimpleFreeSwinging() {
        Dynamics<N4, N1> dynamics = new CartPoleSimple(LENGTH);
        Integrator<N4, N1> integrator = new RK4Integrator<>();
        ControlLaw<N4, N1> control = new ConstantControl<>(VecBuilder.fill(0));
        Simulator<N4, N1> sim = new Simulator<>(
                DT, ET, dynamics::xdot, integrator, control::f);
        Vector<N4> initial = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(initial);
        XYDataset d1 = ChartUtil.xy("x", sim.t(), sim.series(0));
        XYDataset d2 = ChartUtil.xy("theta", sim.t(), sim.series(2));
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(LENGTH, sim)), SCALE);
    }

    @Test
    void testSimpleWithFeedback() {
        Dynamics<N4, N1> dynamics = new CartPoleSimple(LENGTH);
        Integrator<N4, N1> integrator = new RK4Integrator<>();
        // trial and error
        Matrix<N1, N4> K = VecBuilder.fill(-2, -2, -30, -7).transpose();
        ControlLaw<N4, N1> control = new ProportionalFeedback<>(K);
        Simulator<N4, N1> sim = new Simulator<>(
                DT, ET, dynamics::xdot, integrator, control::f);
        Vector<N4> initial = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(initial);
        XYDataset d1 = ChartUtil.xy("x", sim.t(), sim.series(0));
        XYDataset d2 = ChartUtil.xy("theta", sim.t(), sim.series(2));
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(LENGTH, sim)), SCALE);
    }

    @Test
    void testFreeSwinging() {
        Dynamics<N4, N1> dynamics = new CartPoleWithFrictionAndInertia(LENGTH);
        Integrator<N4, N1> integrator = new RK4Integrator<>();
        ControlLaw<N4, N1> control = new ConstantControl<>(VecBuilder.fill(0));
        Simulator<N4, N1> sim = new Simulator<>(
                DT, ET, dynamics::xdot, integrator, control::f);
        Vector<N4> initial = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(initial);
        XYDataset d1 = ChartUtil.xy("x", sim.t(), sim.series(0));
        XYDataset d2 = ChartUtil.xy("theta", sim.t(), sim.series(2));
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(LENGTH, sim)), SCALE);
    }

    @Test
    void testWithFeedback() {
        Dynamics<N4, N1> dynamics = new CartPoleWithFrictionAndInertia(LENGTH);
        Integrator<N4, N1> integrator = new RK4Integrator<>();
        // K from trial and error
        Matrix<N1, N4> K = VecBuilder.fill(-2, -2, -30, -7).transpose();
        ControlLaw<N4, N1> control = new ProportionalFeedback<>(K);
        Simulator<N4, N1> sim = new Simulator<>(
                DT, ET, dynamics::xdot, integrator, control::f);
        Vector<N4> initial = VecBuilder.fill(0, 0, 0.1, 0);
        sim.run(initial);
        XYDataset d1 = ChartUtil.xy("x", sim.t(), sim.series(0));
        XYDataset d2 = ChartUtil.xy("theta", sim.t(), sim.series(2));
        ChartUtil.plotStacked(d1, d2);
        ChartUtil.plotOverlay(List.of(CartPoleChart.make(LENGTH, sim)), SCALE);
    }
}
