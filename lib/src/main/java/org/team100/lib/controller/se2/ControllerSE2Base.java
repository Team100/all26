package org.team100.lib.controller.se2;

import org.team100.lib.geometry.se2.DeltaSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlSE2Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.GlobaDeltaSE2Logger;
import org.team100.lib.logging.LoggerFactory.ModelSE2Logger;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.state.VelocityControlSE2;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Base class for SE(2) controllers.
 * 
 * Implements error calculation and tolerances.
 */
public abstract class ControllerSE2Base implements ControllerSE2 {

    private final ModelSE2Logger m_log_measurement;
    private final ModelSE2Logger m_log_currentReference;
    private final ControlSE2Logger m_log_nextReference;

    private final GlobaDeltaSE2Logger m_log_position_error;
    private final VelocitySE2Logger m_log_velocity_error;
    /** Error in cartesian distance, i.e. hypot(x, y). */
    private final DoubleLogger m_log_cartesianPositionError;
    /** Error in cartesian velocity, i.e. hypot(vx, vy). */
    private final DoubleLogger m_log_cartesianVelocityError;

    private final BooleanLogger m_log_atPositionReference;
    private final BooleanLogger m_log_atVelocityReference;
    private final BooleanLogger m_log_atReference;

    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;
    private final double m_omegaTolerance;

    /** The position error calculated in the most-recent call to calculate. */
    private DeltaSE2 m_positionError;
    /** The velocity error calculated in the most-recent call to calculate. */
    private VelocitySE2 m_velocityError;

    public ControllerSE2Base(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        LoggerFactory log = parent.type(this);

        m_log_measurement = log.modelSE2Logger(Level.DEBUG, "measurement");
        m_log_currentReference = log.modelSE2Logger(Level.DEBUG, "current reference");
        m_log_nextReference = log.controlSE2Logger(Level.DEBUG, "next reference");

        m_log_position_error = log.DeltaSE2Logger(Level.TRACE, "position error");
        m_log_velocity_error = log.VelocitySE2Logger(Level.TRACE, "velocity error");
        m_log_cartesianPositionError = log.doubleLogger(Level.TRACE, "cartesian position error");
        m_log_cartesianVelocityError = log.doubleLogger(Level.TRACE, "cartesian velocity error ");

        m_log_atPositionReference = log.booleanLogger(Level.TRACE, "at position reference");
        m_log_atVelocityReference = log.booleanLogger(Level.TRACE, "at velocity reference");
        m_log_atReference = log.booleanLogger(Level.TRACE, "at reference");

        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;
        m_omegaTolerance = omegaTolerance;
    }

    @Override
    public VelocityControlSE2 calculate(
            ModelSE2 measurement,
            ModelSE2 currentReference,
            ControlSE2 nextReference) {
        m_log_measurement.log(() -> measurement);
        m_log_currentReference.log(() -> currentReference);
        m_log_nextReference.log(() -> nextReference);
        m_positionError = positionError(measurement, currentReference);
        m_velocityError = velocityError(measurement, currentReference);
        return calculate100(m_positionError, m_velocityError, nextReference);
    }

    /**
     * @param positionError current reference minus current measurement
     * @param velocityError current reference minus current measurement
     * @param nextReference velocity for dt from now
     * @returns control output for the period during dt
     */
    public abstract VelocityControlSE2 calculate100(
            DeltaSE2 positionError,
            VelocitySE2 velocityError,
            ControlSE2 nextReference);

    /**
     * Uses the position and velocity errors from the previous call to calculate().
     */
    @Override
    public boolean atReference() {
        if (m_positionError == null || m_velocityError == null)
            return false;
        boolean atReference1 = positionOK(m_positionError) && velocityOK(m_velocityError);
        m_log_atReference.log(() -> atReference1);
        return atReference1;
    }

    /** True if cartesian and rotation position errors are within tolerance. */
    boolean positionOK(DeltaSE2 positionError) {
        double cartesian = positionError.getTranslation().getNorm();
        m_log_cartesianPositionError.log(() -> cartesian);
        double rotation = Math.abs(positionError.getRotation().getRadians());
        boolean withinTolerance = cartesian < m_xTolerance && rotation < m_thetaTolerance;
        m_log_atPositionReference.log(() -> withinTolerance);
        return withinTolerance;
    }

    /** True if cartesian and rotation velocity errors are within tolerance. */
    boolean velocityOK(VelocitySE2 velocityError) {
        double cartesian = velocityError.norm();
        m_log_cartesianVelocityError.log(() -> cartesian);
        double rotation = Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians());
        boolean withinTolerance = cartesian < m_xDotTolerance && rotation < m_omegaTolerance;
        m_log_atVelocityReference.log(() -> withinTolerance);
        return withinTolerance;
    }

    /**
     * Wraps heading.
     */
    DeltaSE2 positionError(ModelSE2 measurement, ModelSE2 currentReference) {
        DeltaSE2 err = DeltaSE2.delta(measurement.pose(), currentReference.pose());
        m_log_position_error.log(() -> err);
        return err;
    }

    /**
     * Velocity does not wrap.
     */
    VelocitySE2 velocityError(ModelSE2 measurement, ModelSE2 currentReference) {
        VelocitySE2 err = currentReference.velocity().minus(measurement.velocity());
        m_log_velocity_error.log(() -> err);
        return err;
    }

}
