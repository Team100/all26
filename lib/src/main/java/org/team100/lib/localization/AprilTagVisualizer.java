package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.StateSE2;
import org.team100.lib.util.TrailingHistory;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.util.struct.StructBuffer;

/**
 * Listens for camera updates, uses the robot's current pose estimate to paint
 * the tag's "apparent position" on the Field2d widget, and in AdvantageScope
 * using the Vision Target feature.
 * 
 * Each apparent tag position should match the actual position, if the cameras
 * are calibrated correctly, and if the camera/robot clocks are in sync.
 * 
 * This is kinda useful for debugging, but it's not *that* useful. It used to be
 * part of AprilTagCornerRobotLocalizer.
 */
public class AprilTagVisualizer {
    record Measurement(double timestamp, Pose3d pose) {
    }

    /** Maximum age of the sights we publish for diagnosis. */
    private static final double HISTORY_DURATION = 1.0;
    private final CameraReader<BlipWithCorners> m_reader;
    private final AprilTagVisualizationTranslator m_translator;
    private final TrailingHistory<Pose3d> m_allTags;
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    private final DoubleArrayLogger m_log_allTags;

    public AprilTagVisualizer(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            DoubleFunction<StateSE2> history,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        LoggerFactory log = parent.type(this);
        m_reader = new CameraReader<>("vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
        m_translator = new AprilTagVisualizationTranslator(
                log, history, layout, alliance);
        m_allTags = new TrailingHistory<>();
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_log_allTags = fieldLogger.doubleArrayLogger(Level.DEBUG, "all tags");
        LogPoller.register(this::update);
    }

    private void update() {
        List<Measurement> measurements = read();
        consumeMeasurements(measurements);
    }

    private List<Measurement> read() {
        // Read all the pending input.
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();
        List<Measurement> measurements = new ArrayList<>();
        for (CameraReader.Record<BlipWithCorners> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    private void consumeMeasurements(List<Measurement> measurements) {
        // Clean the history, relative to the current moment.
        // Previously, eviction only occurred when the robot could see something.
        double deadline = Takt.get() - HISTORY_DURATION;
        m_allTags.evict(deadline);
        for (Measurement m : measurements) {
            m_allTags.add(m.timestamp, m.pose);
        }
        // Show the tags on the Field2d widget and AdvantageScope.
        m_pub_tags.set(m_allTags.getAll().toArray(new Pose3d[0]));
        m_log_allTags.log(
                () -> m_allTags.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(
                                x.getX(),
                                x.getY(),
                                x.toPose2d().getRotation().getDegrees()))
                        .toArray());
    }

}
