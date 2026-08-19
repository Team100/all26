package org.team100.lib.network;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import org.wpilib.networktables.MultiSubscriber;
import org.wpilib.networktables.NetworkTableEvent;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableListenerPoller;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.networktables.ValueEventData;
import org.wpilib.util.struct.StructBuffer;
import org.wpilib.system.RobotController;

/**
 * The server end of the Sync prototcol.
 */
public class Sync implements Runnable {
    private static final int QUEUE_DEPTH = 10;
    private final NetworkTableInstance inst;
    private final StructBuffer<SyncRequest> m_buf;
    private final NetworkTableListenerPoller m_poller;
    private final Map<String, StructPublisher<SyncReply>> m_pubmap;

    public Sync(NetworkTableInstance i) {
        inst = i;
        m_buf = StructBuffer.create(SyncRequest.struct);
        m_poller = new NetworkTableListenerPoller(inst);

        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { "sync" },
                        PubSubOption.KEEP_DUPLICATES,
                        PubSubOption.pollStorage(QUEUE_DEPTH)),
                EnumSet.of(NetworkTableEvent.Kind.VALUE_ALL));
        m_pubmap = new HashMap<>();

    }

    /**
     * Reply if a message is waiting.
     */
    @Override
    public void run() {
        for (NetworkTableEvent e : m_poller.readQueue()) {
            // these events might be for any camera
            ValueEventData valueEventData = e.valueData;
            NetworkTableValue ntValue = valueEventData.value;
            String name = valueEventData.getTopic().getName();
            String[] fields = name.split("/");
            // for now,
            // key is "sync/ID/request" or "sync/ID/reply"
            // String type = fields[1];
            if (fields.length != 3) {
                System.out.printf("WARNING: weird event name: %s\n", name);
                continue;
            }
            String cameraId = fields[1];
            if (fields[2].equals("request")) {
                // reply to this request
                byte[] valueBytes = ntValue.getRaw();
                if (valueBytes.length == 0) {
                    // this should never happen, but it does, very occasionally.
                    continue;
                }

                SyncRequest request;
                try {
                    request = m_buf.read(valueBytes);
                } catch (RuntimeException ex) {
                    System.out.printf("WARNING: decoding failed for name: %s\n", name);
                    continue;
                }

                long org = request.org();
                long now = RobotController.getMonotonicTime();
                StructPublisher<SyncReply> p = m_pubmap.computeIfAbsent(
                        cameraId,
                        x -> inst.getStructTopic(
                                "sync/" + x + "/reply", SyncReply.struct).publish());
                p.set(new SyncReply(org, now, now));
                inst.flush();
            }
        }
    }
}
