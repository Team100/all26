package org.team100.lib.localization;

import java.util.Map;

import org.team100.lib.state.StateSE2;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Choose an implementation based on a flag at runtime.
 * 
 * Used to select estimators: the "fusor" one or the "GTSAM" one.
 */
public class StateEstimatorProxy implements StateEstimator {
    enum EstimatorChoice {
        A, B
    }

    private final StateEstimator m_A;
    private final StateEstimator m_B;
    private final SendableChooser<EstimatorChoice> m_chooser;
    private EstimatorChoice m_choice;

    public StateEstimatorProxy(StateEstimator a, StateEstimator b) {
        m_A = a;
        m_B = b;
        m_chooser = new NamedChooser<>("State Estimator");
        m_chooser.setDefaultOption(EstimatorChoice.A.name(), EstimatorChoice.A);
        m_chooser.addOption(EstimatorChoice.B.name(), EstimatorChoice.B);
        m_choice = EstimatorChoice.A;
        SmartDashboard.putData(m_chooser);
        m_chooser.onChange(this::update);
    }

    private void update(EstimatorChoice choice) {
        m_choice = choice;
    }

    @Override
    public StateSE2 getState(double timestampS) {
        return switch (m_choice) {
            case A -> m_A.getState(timestampS);
            case B -> m_B.getState(timestampS);
            default -> throw new IllegalStateException();
        };
    }

    @Override
    public void reset(Pose2d pose, IsotropicNoiseSE2 noise) {
        switch (m_choice) {
            case A -> m_A.reset(pose, noise);
            case B -> m_B.reset(pose, noise);
            default -> throw new IllegalStateException();
        }
    }

    @Override
    public void setHeedRadiusM(double heedRadiusM) {
        switch (m_choice) {
            case A -> m_A.setHeedRadiusM(heedRadiusM);
            case B -> m_B.setHeedRadiusM(heedRadiusM);
            default -> throw new IllegalStateException();
        }
    }

    @Override
    public Map<Double, SwerveState> all() {
        return switch (m_choice) {
            case A -> m_A.all();
            case B -> m_B.all();
            default -> throw new IllegalStateException();
        };
    }

    @Override
    public void close() {
        m_chooser.close();
    }

}
