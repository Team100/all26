package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Solution for infinite muzzle velocity, for testing.
 * 
 * Azimuth is always the bearing.
 * Azimuth velocity is the apparent target motion.
 * Elevation is specified by the parameters.
 */
public class LaserSolver implements Solver {

    private final DoubleFunction<Optional<FiringParameters>> m_rangeToParams;

    public LaserSolver(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_rangeToParams = rangeToParams;
    }

    @Override
    public Optional<Solution> solve(ModelSE2 state, StateR2 target) {
        Translation2d robotPosition = state.translation();
        Translation2d targetPosition = target.position();

        // Target relative to robot
        Translation2d T0 = targetPosition.minus(robotPosition);
        double rangeM = T0.getNorm();

        Optional<FiringParameters> oParams = m_rangeToParams.apply(rangeM);
        if (oParams.isEmpty())
            return Optional.empty();
        FiringParameters params = oParams.get();

        // Does not include target velocity in solution velocity.
        Rotation2d absoluteBearing = TargetUtil.absoluteBearing(
                robotPosition, targetPosition);
        double azimuthVelocity = TargetUtil.targetMotion(
                state, targetPosition);
        return Optional.of(new Solution(
                absoluteBearing,
                azimuthVelocity,
                params));
    }

}
