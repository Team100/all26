package org.team100.lib.network;

import java.util.function.ObjDoubleConsumer;

import org.team100.lib.camera.Camera;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.Blip;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.StructBuffer;

/** Listen to raw tag input from the cameras, for testing. */
public class RawTags extends CameraReader<Blip> {
    /**
     * The difference between the current instant and the instant of the blip,
     * i.e. this is the time we look up in the pose buffer.
     */
    private final DoubleLogger m_log_lag;
    private final ObjDoubleConsumer<Transform3d> m_sink;

    public RawTags(LoggerFactory parent, ObjDoubleConsumer<Transform3d> sink) {
        super(parent, "vision", "blips", StructBuffer.create(Blip.struct));
        LoggerFactory log = parent.type(this);
        m_log_lag = log.doubleLogger(Level.TRACE, "lag (sec)");
        m_sink = sink;
    }

    @Override
    protected void perValue(Camera camera, Blip[] value) {
        for (Blip b : value) {
            double timeSec = (double) b.getTimestamp() / 1e6;
            m_log_lag.log(() -> Takt.get() - timeSec);
            m_sink.accept(b.blipToTransform(), timeSec);
        }
    }

}
