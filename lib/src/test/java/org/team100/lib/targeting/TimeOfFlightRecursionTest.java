package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.TimeOfFlightRecursion.Looper;
import org.team100.lib.targeting.TimeOfFlightRecursion.Looper.LoopSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TimeOfFlightRecursionTest {
    private static final double DELTA = 0.001;

    Optional<FiringParameters> solver(double x) {
        // tof == range/3 => constant 3 m/s velocity
        return Optional.of(new FiringParameters(x, 1, 1, x / 3));
    }

    DoubleFunction<Optional<FiringParameters>> ir = this::solver;

    /** Motionless solves in one step. */
    @Test
    void testMotionlessParabolicSteps() {
        Translation2d T0 = new Translation2d(2, 0);
        VelocityR2 vT = VelocityR2.ZERO;
        Looper looper = new Looper(ir, T0, vT);
        double targetTOF = ir.apply(T0.getNorm()).get().tof();
        assertEquals(0.666, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF).get();
        assertEquals(0.666, soln.params().tof(), DELTA);
    }

    @Test
    void testMotionlessParabolic() {
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.01);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        VelocityR2 targetVelocity = VelocityR2.ZERO;
        Optional<Solution> o = tofr.solve(
                new ModelSE2(),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();
        assertEquals(0.666, x.parameters().tof(), DELTA);
        assertEquals(0, x.azimuth().getRadians(), DELTA);
        assertEquals(1, x.parameters().elevation(), DELTA);
    }

    /**
     * Takes quite a few iterations for these inputs, chasing
     * the target which is moving away.
     */
    @Test
    void testAwaySteps() {
        Translation2d T0 = new Translation2d(2, 0);
        VelocityR2 vT = new VelocityR2(2, 0);
        Looper looper = new Looper(ir, T0, vT);

        double targetTOF = ir.apply(T0.getNorm()).get().tof();
        assertEquals(0.666, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF).get();
        assertEquals(1.111, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.407, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.604, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.737, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.824, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.883, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.922, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.948, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.965, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.977, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.985, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.990, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.993, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.995, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.997, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(1.998, soln.params().tof(), DELTA);
    }

    /**
     * This takes a whole lot of iterations to match the shooting method, note the
     * tight tolerance.
     */
    @Test
    void testAwayFromTarget() {
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.0001);
        // driving away from the target
        VelocityR2 robotVelocity = new VelocityR2(-2, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        VelocityR2 targetVelocity = VelocityR2.ZERO;

        Optional<Solution> o = tofr.solve(
                new ModelSE2(
                        new Pose2d(),
                        new VelocitySE2(robotVelocity.x(), robotVelocity.y(), 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), DELTA);
        assertEquals(2, x.parameters().tof(), 0.006);
    }


    @Test
    void testHighAwayFromTarget() {
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.0001);
        // driving away from the target
        VelocityR2 robotVelocity = new VelocityR2(-1, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        VelocityR2 targetVelocity = VelocityR2.ZERO;

        Optional<Solution> o = tofr.solve(
                new ModelSE2(
                        new Pose2d(),
                        new VelocitySE2(robotVelocity.x(), robotVelocity.y(), 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution solution = o.orElseThrow();
        assertEquals(0, solution.azimuth().getRadians(), DELTA);
        assertEquals(1, solution.parameters().tof(), 0.006);
    }

    /**
     * Converges a bit faster than the "away" case.
     */
    @Test
    void testStrafingSteps() {
        Translation2d T0 = new Translation2d(2, 0);
        VelocityR2 vT = new VelocityR2(0, -2);

        Looper looper = new Looper(ir, T0, vT);

        double targetTOF = ir.apply(T0.getNorm()).get().tof();
        assertEquals(0.666, targetTOF, DELTA);
        LoopSolution soln = looper.step(targetTOF).get();
        assertEquals(0.801, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(0.854, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(0.877, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(0.887, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(0.891, soln.params().tof(), DELTA);

        targetTOF = soln.params().tof();
        soln = looper.step(targetTOF).get();
        assertEquals(0.893, soln.params().tof(), DELTA);
    }

    @Test
    void testStrafing() {
        TimeOfFlightRecursion tofr = new TimeOfFlightRecursion(ir, 0.001);
        // driving to the left
        VelocityR2 robotVelocity = new VelocityR2(0, 2);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        VelocityR2 targetVelocity = VelocityR2.ZERO;

        Optional<Solution> o = tofr.solve(
                new ModelSE2(new Pose2d(),
                        new VelocitySE2(robotVelocity.x(), robotVelocity.y(), 0)),
                new StateR2(targetPosition, targetVelocity));
        Solution x = o.orElseThrow();
        assertEquals(-0.729, x.azimuth().getRadians(), 0.002);
        assertEquals(0.894, x.parameters().tof(), 0.003);
    }

}
