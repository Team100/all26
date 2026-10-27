package org.team100.lib.network;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ObjDoubleConsumer;

import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.Blip;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.StructBuffer;

/** Listen to raw tag input from the cameras, for testing. */
public class RawTags {
    record TagMeasurement(double timestamp, Transform3d transform) {
    }

    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final CameraReader<Blip> m_reader;
    private final RawTagTranslator m_translator;
    private final DoubleLogger m_log_lag;
    private final ObjDoubleConsumer<Transform3d> m_sink;

    public RawTags(LoggerFactory parent, ObjDoubleConsumer<Transform3d> sink) {
        m_reader = new CameraReader<>("vision", "blips",
                StructBuffer.create(Blip.struct));
        m_translator = new RawTagTranslator();
        LoggerFactory log = parent.type(this);
        m_log_lag = log.doubleLogger(Level.TRACE, "lag (sec)");
        m_sink = sink;
    }

    public void update() {
        List<TagMeasurement> measurements = read();
        consumeMeasurements(measurements);
    }

    private List<TagMeasurement> read() {
        List<CameraReader.Record<Blip>> records = m_reader.getRecords();
        List<TagMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<Blip> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    private void consumeMeasurements(List<TagMeasurement> measurements) {
        for (TagMeasurement m : measurements) {
            m_log_lag.log(() -> Takt.get() - m.timestamp);
            m_sink.accept(m.transform, m.timestamp);
        }
    }

}
