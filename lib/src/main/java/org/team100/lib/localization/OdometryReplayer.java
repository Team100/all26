package org.team100.lib.localization;

import java.util.Map;

import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Replay odometry after the sample time, modifying the history.
 */
public class OdometryReplayer {
    private static final boolean DEBUG = false;
    private final SwerveHistory m_history;
    private final OdometryEstimator m_updater;

    public OdometryReplayer(SwerveHistory history, OdometryEstimator updater) {
        m_history = history;
        m_updater = updater;
    }

    public void replay(double sampleTime) {
        if (DEBUG)
            System.out.printf("==== REPLAY FOR TIME %f\n", sampleTime);
        // Note the exclusive tailmap: we don't see the entry at timestamp.
        for (Map.Entry<Double, SwerveState> entry : m_history.exclusiveTailMap(sampleTime).entrySet()) {
            double timestamp = entry.getKey();
            SwerveState value = entry.getValue();
            Rotation2d gyroYaw = value.gyroYaw();
            SwerveModulePositions positions = value.positions();
            SwerveState s = m_updater.estimate(timestamp, gyroYaw, positions);
            if (s != null)
                m_history.put(timestamp, s);
        }
        if (DEBUG)
            System.out.printf("==== DONE REPLAYING FOR TIME %f\n", sampleTime);
    }

}
