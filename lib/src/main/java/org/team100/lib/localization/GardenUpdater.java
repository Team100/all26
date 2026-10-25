package org.team100.lib.localization;

import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Read all pose estimate inputs at once and call the gardener.
 */
public class GardenUpdater {

    private final Gyro m_gyro;
    private final Supplier<SwerveModulePositions> m_positions;
    private final Supplier<List<VisionMeasurement>> m_vision;
    private final HistoryGardener m_gardener;

    public GardenUpdater(
            Gyro gyro,
            Supplier<SwerveModulePositions> positions,
            Supplier<List<VisionMeasurement>> vision,
            HistoryGardener gardener) {
        m_gyro = gyro;
        m_positions = positions;
        m_vision = vision;
        m_gardener = gardener;
    }

    /**
     * Add current measurements for odometry and gyro.
     * Add pending vision measurements.
     * Sweep the garden.
     */
    public void update(double timestamp) {
        // Odometry measurement has one value per update at the current time
        SwerveModulePositions positions = m_positions.get();
        Rotation2d yawNWU = m_gyro.getYawNWU();
        m_gardener.putOdometry(timestamp, yawNWU, positions);
        // Vision may have any number of updates at arbitrary times
        for (VisionMeasurement v : m_vision.get()) {
            m_gardener.putVision(v.timestamp(), v.noisyMeasurement());
        }
        m_gardener.sweep();
    }

}
