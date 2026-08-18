package org.team100.lib.reference.se2;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.r2.StateR2;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.targeting.Solution;
import org.team100.lib.targeting.Solver;

import edu.wpi.first.math.geometry.Translation2d;

/** Use a delegate for cartesian reference, and a target for rotation. */
public class TargetReferenceSE2 implements ReferenceSE2 {

    private final ReferenceSE2 m_delegate;
    private final Solver m_solver;
    private final BooleanSupplier m_override;

    public TargetReferenceSE2(
            ReferenceSE2 delegate,
            Solver solver,
            BooleanSupplier override) {
        m_delegate = delegate;
        m_solver = solver;
        m_override = override;
    }

    public void initialize(ModelSE2 measurement) {
        m_delegate.initialize(measurement);
    }

    public ModelSE2 current() {
        return override(m_delegate.current());
    }

    public ControlSE2 next() {
        return override(m_delegate.next());
    }

    public boolean done() {
        return m_delegate.done();
    }

    public ModelSE2 goal() {
        return m_delegate.goal();
    }

    private ModelSE2 override(ModelSE2 model) {
        if (!m_override.getAsBoolean())
            return model;
        Optional<Translation2d> oTarget = FieldConstants2026.TARGET(
                model.translation());
        if (oTarget.isEmpty())
            return model;
        StateR2 target = new StateR2(oTarget.get(), VelocityR2.ZERO);
        Optional<Solution> oSolution = m_solver.solve(model, target);
        if (oSolution.isEmpty())
            return model;
        Solution solution = oSolution.get();
        ModelR1 theta = new ModelR1(
                solution.azimuth().getRadians(),
                solution.azimuthVelocity());
        return new ModelSE2(model.x(), model.y(), theta);
    }

    private ControlSE2 override(ControlSE2 control) {
        if (!m_override.getAsBoolean())
            return control;
        Optional<Translation2d> oTarget = FieldConstants2026.TARGET(
                control.translation());
        if (oTarget.isEmpty())
            return control;
        StateR2 target = new StateR2(oTarget.get(), VelocityR2.ZERO);
        Optional<Solution> oSolution = m_solver.solve(control.model(), target);
        if (oSolution.isEmpty())
            return control;
        Solution solution = oSolution.get();
        ControlR1 theta = new ControlR1(
                solution.azimuth().getRadians(),
                solution.azimuthVelocity());
        return new ControlSE2(control.x(), control.y(), theta);
    }

}
