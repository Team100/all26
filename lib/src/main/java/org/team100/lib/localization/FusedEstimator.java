package org.team100.lib.localization;

import java.util.Map;
import java.util.function.UnaryOperator;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.SwerveLocal;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provides state estimates after updating vision and odometry.
 * 
 * The underlying updaters use "Fusors" and replay.
 */
public class FusedEstimator implements StateEstimator {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;

    private final Gyro m_gyro;
    private final SwerveLocal m_swerveLocal;
    private final SwerveHistory m_history;
    private final AprilTagCornerRobotLocalizer m_localizer;
    private final OdometryUpdater m_odometryUpdate;
    /** Side effect mutates history. */
    private final SideEffect m_localizerCache;
    /** Side effect mutates history. */
    private final SideEffect m_odometryCache;

    public FusedEstimator(LoggerFactory driveLog,
            LoggerFactory fieldLogger,
            SwerveKinodynamics swerveKinodynamics,
            UnaryOperator<Twist2d> odometryNoise,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Gyro gyro,
            SwerveLocal swerveLocal) {
        m_gyro = gyro;
        m_swerveLocal = swerveLocal;
        m_history = new SwerveHistory(
                driveLog,
                0.2,
                gyro.getYawNWU(),
                VariableR1.fromStdDev(0, 1),
                swerveLocal.positions(),
                Pose2d.kZero,
                IsotropicNoiseSE2.high(),
                Takt.get());
        m_odometryUpdate = new OdometryUpdater(
                driveLog,
                swerveKinodynamics,
                gyro,
                m_history,
                swerveLocal::positions,
                odometryNoise,
                false);
        NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                driveLog, m_history, m_odometryUpdate);
        m_localizer = new AprilTagCornerRobotLocalizer(
                driveLog,
                layout,
                visionUpdater,
                DriverStation::getAlliance);
        m_localizerCache = Cache.ofSideEffect(m_localizer::update);
        m_odometryCache = Cache.ofSideEffect(m_odometryUpdate::update);
    }

    public Map<Double, SwerveState> all() {
        return m_history.exclusiveTailMap(0);
    }

    /**
     * Estimate at the given timestamp, after applying any pending updates from
     * vision or odometry.
     * 
     * The estimate is used for many things downstream; noise there is bad.
     * The estimator itself should have enough controls to make the estimate
     * arbitrarily smooth.
     */
    @Override
    public StateSE2 get(double timestampS) {
        // run our dependencies if they haven't already
        m_localizerCache.run();
        m_odometryCache.run();
        final StateSE2 state;
        if (Experiments.INSTANCE.enabled(Experiment.ImputeVelocity)) {
            // Use consecutive poses
            StateSE2 state0 = m_history.get(timestampS - DT);
            StateSE2 state1 = m_history.get(timestampS);
            VelocitySE2 v = VelocitySE2.velocity(
                    state0.pose(),
                    state1.pose(), DT);
            state = new StateSE2(state1.pose(), v);
        } else {
            // Use the history value
            state = m_history.get(timestampS);
        }
        if (DEBUG) {
            System.out.printf("FreshSwerveEstimate.update() estimated pose: %s\n", state);
        }
        return state;
    }

    /**
     * Empty the pose history, reset the servos, add the given pose, and flush the
     * cache.
     */
    @Override
    public void reset(Pose2d pose, IsotropicNoiseSE2 noise) {
        m_history.reset(
                m_swerveLocal.positions(),
                pose,
                noise,
                Takt.get(),
                m_gyro.getYawNWU(),
                VariableR1.fromVariance(0, 1));
        m_localizerCache.reset();
        m_odometryCache.reset();
    }

    /**
     * Tags outside this radius are ignored.
     */
    @Override
    public void setHeedRadiusM(double heedRadiusM) {
        m_localizer.setHeedRadiusM(heedRadiusM);
    }

}
