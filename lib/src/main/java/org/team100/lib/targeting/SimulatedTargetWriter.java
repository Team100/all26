package org.team100.lib.targeting;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleFunction;

import org.team100.frc2025.field.FieldConstants2025;
import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Write simulated targets to Network Tables, so the Targets receiver can pick
 * them up.
 * 
 * This uses a separate client NT instance, so there will be weird delays due to
 * NT rate-limiting -- these are realistic and so should be handled correctly.
 */
public class SimulatedTargetWriter {
    private static final boolean DEBUG = false;

    // camera frame is from 85 ms ago
    private static final double DELAY = 0.085;

    private final Map<Camera, StructArrayPublisher<Target>> m_publishers;
    private final DoubleLogger m_log_poseTimestamp;
    private final List<Camera> m_cameras;
    private final DoubleFunction<ModelSE2> m_history;

    /** For now, a fixed list of targets */
    private final Translation2d[] m_targets;
    /** client instance, not the default */
    private final NetworkTableInstance m_inst;

    public SimulatedTargetWriter(
            LoggerFactory parent,
            List<Camera> cameras,
            DoubleFunction<ModelSE2> history,
            Translation2d[] targets) {
        LoggerFactory log = parent.type(this);
        m_log_poseTimestamp = log.doubleLogger(Level.TRACE, "pose timestamp (s)");
        m_cameras = cameras;
        m_history = history;
        m_targets = targets;
        m_publishers = new HashMap<>();
        // Use a separate instance so that the timestamps are written correctly.
        m_inst = NetworkTableInstance.create();
        m_inst.setServer("localhost");
        m_inst.startClient4("tag_finder24");
        for (Camera camera : m_cameras) {
            // name is "objectVision/{IDENTITY/targets"
            String name = "objectVision/" + camera.getSerial() + "/targets";
            m_publishers.put(
                    camera,
                    m_inst.getStructArrayTopic(
                            name, Target.struct).publish(PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.01),
                                    PubSubOption.sendAll(true)));
        }
    }

    public static Runnable get(LoggerFactory parent, SwerveHistory history) {
        if (RobotBase.isReal()) {
            // Real robots get an empty simulated target detector.
            return () -> {
            };
        }
        // In simulation, we want the real simulated target detector.
        SimulatedTargetWriter tsim = new SimulatedTargetWriter(
                parent,
                List.of(Camera.SIM0, Camera.SIM1, Camera.SIM2, Camera.SIM3),
                history,
                new Translation2d[] {
                        FieldConstants2025.CoralMark.LEFT.value,
                        FieldConstants2025.CoralMark.CENTER.value,
                        FieldConstants2025.CoralMark.RIGHT.value });
        return tsim::update;
    }

    public void update() {
        if (DEBUG)
            System.out.println("simulated target write update");
        // select pose from a little while ago
        double timestampS = Takt.get() - DELAY;
        m_log_poseTimestamp.log(() -> timestampS);
        Pose2d pose = m_history.apply(timestampS).pose();

        for (Map.Entry<Camera, StructArrayPublisher<Target>> entry : m_publishers.entrySet()) {
            Camera camera = entry.getKey();
            StructArrayPublisher<Target> publisher = entry.getValue();
            List<Rotation3d> rot = SimulatedObjectDetector.getRotations(
                    pose, Offset.get(camera).offset(), m_targets);
            if (DEBUG) {
                System.out.printf("rot size %d\n", rot.size());
            }
            // tilt down 45
            // Rotation3d[] rots = new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) };
            Target[] rots = rot.stream().map(x -> new Target(0, x)).toArray(Target[]::new);

            // Use exactly the timestamp used in this history lookup.
            long time = (long) (timestampS * 1000000.0);
            if (DEBUG) {
                System.out.printf("writer timestamp us %d\n", time);
            }
            publisher.set(rots, time);
        }
        m_inst.flushLocal();
        m_inst.flush();
    }

    public void close() {
        m_publishers.values().forEach(p -> p.close());
        m_inst.close();
    }
}
