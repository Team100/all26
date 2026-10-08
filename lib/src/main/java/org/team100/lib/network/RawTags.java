package org.team100.lib.network;

import java.util.List;
import java.util.function.ObjDoubleConsumer;

import org.team100.lib.camera.Camera;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.Blip;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import org.wpilib.math.geometry.Transform3d;
import org.wpilib.util.struct.StructBuffer;

/** Listen to raw tag input from the cameras, for testing. */
public class RawTags {
    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final CameraReader<Blip> m_reader;
    private final DoubleLogger m_log_lag;
    private final ObjDoubleConsumer<Transform3d> m_sink;

    public RawTags(LoggerFactory parent, ObjDoubleConsumer<Transform3d> sink) {
        m_reader = new CameraReader<>("vision", "blips",
                StructBuffer.create(Blip.struct));
        LoggerFactory log = parent.type(this);
        m_log_lag = log.doubleLogger(Level.TRACE, "lag (sec)");
        m_sink = sink;
    }

    public void update() {
        List<CameraReader.Record<Blip>> records = m_reader.getRecords();
        for (CameraReader.Record<Blip> r : records) {
            perValue(r.camera(), r.values());
        }
    }

    protected void perValue(Camera camera, Blip[] value) {
        for (Blip b : value) {
            double timeSec = (double) b.getTimestamp() / 1e6;
            m_log_lag.log(() -> Takt.get() - timeSec);
            m_sink.accept(b.blipToTransform(), timeSec);
        }
    }

}
