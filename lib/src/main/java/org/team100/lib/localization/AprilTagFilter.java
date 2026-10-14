package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.Metrics;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.wpilib.math.geometry.Pose2d;

/**
 * Filter vision measurements to remove unreliable ones.
 */
public class AprilTagFilter {
    private static final boolean DEBUG = false;
    /** Discard results further than this from the previous one. */
    private static final double VISION_CHANGE_TOLERANCE_M = 0.25;

    private final Pose2dLogger m_log_pose;
    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final DoubleLogger m_log_lag;

    /**
     * Remember the previous vision-based pose estimate, so we can measure the
     * distance between consecutive updates, and ignore too-far updates.
     */
    private Pose2d m_prevPose;

    public AprilTagFilter(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_log_pose = log.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = log.doubleLogger(Level.TRACE, "lag");
    }

    public List<VisionMeasurement> filter(List<VisionMeasurement> measurements) {
        List<VisionMeasurement> filteredMeasurements = new ArrayList<>();
        for (VisionMeasurement m : measurements) {
            m_log_lag.log(() -> Takt.get() - m.timestamp());
            Pose2d pose = m.noisyMeasurement().pose();
            m_log_pose.log(() -> pose);

            //////////////////////////////////////////////////////////////////
            ///
            /// Should we use this update?
            ///
            if (Experiments.INSTANCE.enabled(Experiment.IgnoreVision)) {
                // No, vision is off.
                if (DEBUG)
                    System.out.println("Drop update, vision is off.");
                continue;
            }
            if (m_prevPose == null) {
                // No, we need another nearby fix to believe either one.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.println("Need confirmation.");
                continue;
            }
            if (Metrics.translationalDistance(m_prevPose, pose) > VISION_CHANGE_TOLERANCE_M) {
                // No, the new estimate is too far from the previous one.
                m_prevPose = pose;
                if (DEBUG)
                    System.out.println("New estimate is too far away.");
                continue;
            }
            /// Yes, we should use this update.
            /// 
            ///
            m_prevPose = pose;
            filteredMeasurements.add(m);
        }
        return filteredMeasurements;
    }
}
