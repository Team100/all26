package org.team100.lib.targeting;

import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Provides a solver based on the dashboard widget.
 */
public class ProxySolver implements Solver {
    private final SendableChooser<Solver> m_chooser;

    public ProxySolver(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser = new NamedChooser<>("Target Solver");
        addTOFRecursion(rangeToParams);
        addLaser(rangeToParams);
        SmartDashboard.putData(m_chooser);
    }

    /** Keeps tests from conflicting. */
    public void close() {
        m_chooser.close();
    }

    /**
     * Real 3d trajectory.
     * 
     * Iterates on predicted TOF using inverse map.
     */
    private void addTOFRecursion(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser.addOption("TOFR", new TimeOfFlightRecursion(rangeToParams, 0.01));
    }

    /**
     * Two-dimensional solution.
     * 
     * For testing, using the flashlight.
     */
    private void addLaser(DoubleFunction<Optional<FiringParameters>> rangeToParams) {
        m_chooser.setDefaultOption("Laser", new LaserSolver(rangeToParams));
    }

    @Override
    public Optional<Solution> solve(ModelSE2 state, StateR2 target) {
        return m_chooser.getSelected().solve(state, target);
    }
}
