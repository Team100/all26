package org.team100.lib.network;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import org.team100.lib.camera.Camera;
import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableEvent;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableListenerPoller;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.ValueEventData;
import org.wpilib.util.struct.StructBuffer;

/**
 * Reads camera input from network tables, which is always a StructArray.
 * 
 * @param T payload type, e.g. Blip.
 */
public final class CameraReader<T> {
    private static final boolean DEBUG = false;

    public record Record<T>(Camera camera, T[] values) {
    }

    /**
     * Five cameras, 50hz each => 250 hz of updates. Rio runs at 50 hz, so there
     * should be five messages waiting for us each cycle.
     */
    private static final int QUEUE_DEPTH = 10;

    /** e.g. "blips" or "Rotation3d" */
    private final String m_value;
    /** Manages the queue of incoming messages. */
    private final NetworkTableListenerPoller m_poller;
    /** Deserializer used in update(). */
    private final StructBuffer<T> m_buf;

    /**
     * Polls for keys of the form /root/camera_id/value.
     * 
     * @param root  first part of the key, e.g. "vision"
     * @param value last part of the key, e.g. "blips"
     * @param buf   deserializer, e.g. StructBuffer.create(Blip.struct).
     */
    public CameraReader(String root, String value, StructBuffer<T> buf) {
        m_value = value;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { root },
                        PubSubOption.KEEP_DUPLICATES,
                        PubSubOption.pollStorage(QUEUE_DEPTH)),
                EnumSet.of(NetworkTableEvent.Kind.VALUE_ALL));
        m_buf = buf;
    }

    /**
     * Read all queued input and return it as records.
     */
    public List<Record<T>> getRecords() {
        List<Record<T>> records = new ArrayList<>();
        for (NetworkTableEvent e : m_poller.readQueue()) {
            ValueEventData valueEventData = e.valueData;
            NetworkTableValue ntValue = valueEventData.value;
            String name = valueEventData.getTopic().getName();
            if (DEBUG) {
                System.out.printf("poll %s\n", name);
            }
            String[] fields = name.split("/");
            if (fields.length != 3) {
                // System.out.printf("WARNING: weird event name: %s\n", name);
                continue;
            }
            // key is "rootName/cameraId/valueName"
            String cameraId = fields[1];
            if (!fields[2].equals(m_value)) {
                continue;
            }
            if (DEBUG) {
                System.out.println("found value");
            }
            // decode the way StructArrayEntryImpl does
            byte[] valueBytes = ntValue.getRaw();
            if (valueBytes.length == 0) {
                if (DEBUG) {
                    System.out.println("empty message!");
                }
                // this should never happen, but it does, very occasionally.
                continue;
            }
            T[] valueArray;
            try {
                valueArray = m_buf.readArray(valueBytes);
            } catch (RuntimeException ex) {
                System.out.printf("WARNING: decoding failed for name: %s\n", name);
                continue;
            }
            Camera camera = Camera.get(cameraId);
            records.add(new Record<>(camera, valueArray));
        }
        return records;
    }
}
