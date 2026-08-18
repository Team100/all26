package org.team100.lib.controller.r1;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.ModelR1Logger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/**
 * Controls omega to hit a moving target from a moving platform.
 * 
 * Use this to override user or trajectory omega input.
 * 
 * Leads relative target motion.
 */
public class AzimuthController {
    private final DoubleSupplier m_maxOmega;
    private final FeedbackR1 m_thetaController;

    private final ModelR1Logger m_log_thetaGoal;
    private final ModelR1Logger m_log_goal;
    private final DoubleLogger m_log_thetaFB;
    private final DoubleLogger m_log_thetaFF;
    private final DoubleLogger m_log_omega;

    public AzimuthController(
            LoggerFactory parent,
            DoubleSupplier maxOmega,
            FeedbackR1 thetaController) {
        LoggerFactory log = parent.type(this);
        m_log_goal = log.ModelR1Logger(Level.TRACE, "goal");
        m_log_thetaFB = log.doubleLogger(Level.TRACE, "thetaFB");
        m_log_thetaFF = log.doubleLogger(Level.TRACE, "thetaFF");
        m_log_omega = log.doubleLogger(Level.TRACE, "omega");
        m_log_thetaGoal = log.ModelR1Logger(Level.TRACE, "theta goal");
        m_maxOmega = maxOmega;
        m_thetaController = thetaController;
    }

    public void reset() {
        m_thetaController.reset();
    }

    public Double getOmega(ModelR1 thetaMeasurement, ModelR1 thetaGoal) {
        m_log_thetaGoal.log(() -> thetaGoal);
        double thetaFB = getThetaFB(thetaMeasurement, thetaGoal);
        double thetaFF = getThetaFF(thetaGoal);
        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_maxOmega.getAsDouble(),
                m_maxOmega.getAsDouble());
        m_log_omega.log(() -> omega);
        return omega;
    }

    /** Feedback uses the previous goal. */
    private double getThetaFB(ModelR1 thetaMeasurement, ModelR1 thetaGoal) {
        // wrap correctly
        double robotYaw = thetaMeasurement.x();
        double goalYaw = Math100.getMinDistance(robotYaw, thetaGoal.x());
        ModelR1 goal = new ModelR1(goalYaw, thetaGoal.v());
        m_log_goal.log(() -> goal);
        // compute feedback
        double thetaFB = m_thetaController.calculate(thetaMeasurement, goal);
        m_log_thetaFB.log(() -> thetaFB);
        return thetaFB;
    }

    private double getThetaFF(ModelR1 thetaGoal) {
        double thetaFF = thetaGoal.v();
        m_log_thetaFF.log(() -> thetaFF);
        return thetaFF;
    }

}
