package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.network.CameraReader;
import org.wpilib.util.struct.StructBuffer;

/**
 * Reads camera observations, produces robot pose estimates.
 * 
 * Uses the camera-relative tag pose computed by the camera.
 */
public class AprilTagReader {
    private final CameraReader<Blip> m_reader;
    private final AprilTagTranslator m_translator;
    private final AprilTagFilter m_filter;

    public AprilTagReader(
            LoggerFactory parent,
            AprilTagTranslator translator) {
        LoggerFactory log = parent.type(this);
        m_reader = new CameraReader<>(
                "vision", "blips", StructBuffer.create(Blip.struct));
        m_translator = translator;
        m_filter = new AprilTagFilter(log);
    }

    /**
     * Read all pending input, convert into a list of measurements, filter
     * unreliable input, return the filtered list.
     */
    public List<VisionMeasurement> read() {
        return m_filter.filter(readAll());
    }

    /**
     * Read all pending input, translate into measurements, return a list of them.
     */
    private List<VisionMeasurement> readAll() {
        List<CameraReader.Record<Blip>> records = m_reader.getRecords();
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<Blip> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    /**
     * Tags outside this radius are ignored.
     */
    void setHeedRadiusM(double heedRadiusM) {
        m_translator.setHeedRadiusM(heedRadiusM);
    }
}