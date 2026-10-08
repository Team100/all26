package org.team100.lib.localization;

import java.util.Iterator;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.uncertainty.NoisyPose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Updates the whole history based on new and old inputs.
 * 
 * The old way we did this was to add a vision estimate, and then replay
 * odometry after that, and do that over and over for every vision input.
 * This was an evolution of the older WPI estimator.
 * 
 * The new way is to add all the inputs, and then sweep the history one time,
 * to do all the nudging and integrating in one pass.
 * 
 * The repetition in the first way is required because none of the replaying
 * was aware of any subsequent vision input: it just takes the vision-nudged
 * estimate and integrates the odometry. So without this repeated-integration
 * approach, if vision updates were received out-of-order (which was/is common)
 * then the "earlier" ones would end up overwriting the "later" ones.
 */
public class HistoryGardener {
    record Odo(Rotation2d gyro, SwerveModulePositions odo) {
    }

    private final NavigableMap<Double, NoisyPose2d> m_pendingVision;
    private final NavigableMap<Double, Odo> m_pendingOdo;

    private final SwerveHistory m_history;
    private final OdometryUpdater m_odometryUpdater;
    private final NudgingVisionUpdater m_visionUpdater;

    public HistoryGardener(
            SwerveHistory history,
            OdometryUpdater odometryUpdater,
            NudgingVisionUpdater visionUpdater) {
        m_pendingVision = new ConcurrentSkipListMap<>();
        m_pendingOdo = new ConcurrentSkipListMap<>();
        m_history = history;
        m_odometryUpdater = odometryUpdater;
        m_visionUpdater = visionUpdater;
    }

    /** Add pending odometry measurement. */
    public void putOdometry(
            double timeSec,
            Rotation2d gyroYaw,
            SwerveModulePositions positions) {
        m_pendingOdo.put(timeSec, new Odo(gyroYaw, positions));
    }

    /** Add pending vision measurement. */
    public void putVision(
            double timeSec,
            NoisyPose2d measurement) {
        m_pendingVision.put(timeSec, measurement);
    }

    /** Handle pending updates in time order. */
    public void sweep() {
        Iterator<Entry<Double, Odo>> odoIter = m_pendingOdo.entrySet().iterator();
        Iterator<Entry<Double, NoisyPose2d>> visionIter = m_pendingVision.entrySet().iterator();

        Entry<Double, Odo> odo = odoIter.hasNext() ? odoIter.next() : null;
        Entry<Double, NoisyPose2d> vision = visionIter.hasNext() ? visionIter.next() : null;

        while (odo != null && vision != null) {
            if (odo.getKey() < vision.getKey()) {
                m_odometryUpdater.put(odo.getKey(), odo.getValue().gyro, odo.getValue().odo);
                odo = odoIter.hasNext() ? odoIter.next() : null;
            } else {
                m_visionUpdater.put(vision.getKey(), vision.getValue());
                vision = visionIter.hasNext() ? visionIter.next() : null;
            }
        }
        // catch the remaining; one of these will work.
        while (odo != null) {
            m_odometryUpdater.put(odo.getKey(), odo.getValue().gyro, odo.getValue().odo);
            odo = odoIter.hasNext() ? odoIter.next() : null;
        }
        while (vision != null) {
            m_visionUpdater.put(vision.getKey(), vision.getValue());
            vision = visionIter.hasNext() ? visionIter.next() : null;
        }

    }
}
