package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.TimeOfFlightRecursion.Looper.LoopSolution;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Time-of-flight recursion, as described by @oblarg is an iterative approach
 * that works as follows:
 * 
 * 1. shoot at the target location
 * 2. evolve the target location based on the TOF of that shot
 * 3. repeat until the shooting solution doesn't change
 * 
 * see
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815/21?u=truher
 */
public class TimeOfFlightRecursion implements Solver {
    private static final boolean DEBUG = false;

    /** Look up solution parameters for range. */
    private final DoubleFunction<Optional<FiringParameters>> m_rangeToParams;
    /** Solution TOF tolerance, seconds. */
    private final double m_tolerance;

    /**
     * @param rangeToParams FiringParameters as a function of desired range
     * @param tolerance     complete when the solution doesn't change more than
     *                      this, in seconds
     */
    public TimeOfFlightRecursion(
            DoubleFunction<Optional<FiringParameters>> rangeToParams,
            double tolerance) {
        m_rangeToParams = rangeToParams;
        m_tolerance = tolerance;
    }

    /** This is a separate class to make it easier to test. */
    static class Looper {
         record LoopSolution(
                FiringParameters params, Translation2d targetPositionAtTOF) {
        }

        private final DoubleFunction<Optional<FiringParameters>> m_rangeToParams;
        /** Target position relative to robot. */
        private final Translation2d m_T0;
        /** Target velocity relative to robot. */
        private final VelocityR2 m_vT;

        public Looper(
                DoubleFunction<Optional<FiringParameters>> rangeToParams,
                Translation2d T0,
                VelocityR2 vT) {
            m_rangeToParams = rangeToParams;
            m_T0 = T0;
            m_vT = vT;
        }

        Optional<LoopSolution> step(Double targetTOF) {

            // where is the target at the specified TOF?
            Translation2d targetPositionAtTOF = m_vT.integrate(m_T0, targetTOF);
            double rangeAtTOF = targetPositionAtTOF.getNorm();

            // What gun elevation gets to that range, and what is the
            // ball TOF to get there?
            Optional<FiringParameters> oParams = m_rangeToParams.apply(rangeAtTOF);
            if (oParams.isEmpty()) {
                if (DEBUG) {
                    System.out.printf("No solution for range %f\n", rangeAtTOF);
                    complain(m_vT, m_T0);
                }
                return Optional.empty();
            }
            FiringParameters params = oParams.get();
            LoopSolution var2 = new LoopSolution(
                    params, targetPositionAtTOF);
            if (DEBUG)
                System.out.printf("range %f elevation %f tof %f\n",
                        var2.targetPositionAtTOF.getNorm(),
                        var2.params.elevation(), var2.params.tof());
            return Optional.of(var2);
        }
    }

    private static void complain(VelocityR2 vT, Translation2d T0) {
        System.out.printf("vT (%f, %f) T0 (%f, %f)\n",
                vT.x(), vT.y(), T0.getX(), T0.getY());
    }

    @Override
    public Optional<Solution> solve(ModelSE2 state, StateR2 target) {
        final Translation2d robotPosition = state.translation();
        final VelocityR2 robotVelocity = state.velocityR2();

        // Target relative to robot
        Translation2d T0 = target.position().minus(robotPosition);
        if (DEBUG) {
            System.out.printf("robot %s target %s\n",
                    StrUtil.transStr(robotPosition),
                    StrUtil.transStr(target.position()));
        }
        double rangeM = T0.getNorm();
        // Target velocity relative to robot
        VelocityR2 vT = target.velocity().minus(robotVelocity);

        Looper looper = new Looper(m_rangeToParams, T0, vT);

        // Initial guess is the initial location.
        Optional<FiringParameters> initial = m_rangeToParams.apply(rangeM);
        if (initial.isEmpty()) {
            if (DEBUG)
                System.out.printf("No initial solution for %f\n", rangeM);
            return Optional.empty();
        }
        double targetTOF = initial.get().tof();

        for (int i = 0; i < 100; ++i) {
            Optional<LoopSolution> oSoln = looper.step(targetTOF);
            if (oSoln.isEmpty()) {
                // Note: if *any* step along the way is invalid,
                // this method fails, even if the end point might
                // be valid.
                // TODO: use better initial guesses to avoid that.
                if (DEBUG) {
                    System.out.printf("No solution for target TOF %f\n", targetTOF);
                }
                return Optional.empty();
            }
            LoopSolution soln = oSoln.get();
            double ballTOF = soln.params.tof();
            if (MathUtil.isNear(targetTOF, ballTOF, m_tolerance)) {
                // Found a good solution!
                double targetMotion = TargetUtil.targetMotion(
                        state, soln.targetPositionAtTOF);
                return Optional.of(new Solution(
                        soln.targetPositionAtTOF.getAngle(),
                        targetMotion,
                        soln.params));
            }
            // try again with a new target TOF guess
            targetTOF = ballTOF;
        }
        return Optional.empty();
    }

}
