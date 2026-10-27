package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.network.CameraReader;

import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Reads camera observations, produces robot pose estimates.
 * 
 * Uses the corners of observed AprilTags, not the camera-relative tag pose.
 */
public class AprilTagCornerReader {
    private final CameraReader<BlipWithCorners> m_reader;
    private final AprilTagCornerTranslator m_translator;
    private final AprilTagFilter m_filter;

    public AprilTagCornerReader(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        m_reader = new CameraReader<>(
                "vision", "blips_with_corners", StructBuffer.create(BlipWithCorners.struct));
        m_translator = new AprilTagCornerTranslator(log, layout, alliance);
        m_filter = new AprilTagFilter(log);
    }

    /**
     * Read all pending input, convert into measurements, filter unreliable input,
     * return a list of measurements.
     */
    public List<VisionMeasurement> read() {
        return m_filter.filter(readAll());
    }

    /** Read all pending input and return a list of measurements */
    private List<VisionMeasurement> readAll() {
        // camera inputs
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<BlipWithCorners> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    /**
     * Tags outside this radius are ignored.
     * 
     * TODO: remove this, the noise model should take care of it.
     */
    void setHeedRadiusM(double heedRadiusM) {
        m_translator.setHeedRadiusM(heedRadiusM);
    }

}