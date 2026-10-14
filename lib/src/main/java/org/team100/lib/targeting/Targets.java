package org.team100.lib.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.stream.DoubleStream;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.r2.CentroidR2;
import org.team100.lib.geometry.r2.NearR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.state.StateSE2;
import org.team100.lib.util.CoalescingCollection;
import org.team100.lib.util.TrailingHistory;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 */
public class Targets {
    private static final boolean DEBUG = false;

    public record TargetMeasurement(double timestamp, Translation2d location) {
    }

    /** Forget sights older than this. */
    private static final double HISTORY_DURATION = 1.0;
    /** Targets closer than this to each other are combined */
    private static final double NEARNESS_THRESHOLD = 0.15;

    private final CameraReader<Target> m_reader;
    private final DoubleLogger m_log_age;
    private final DoubleLogger m_log_poseTimestamp;

    /**
     * Ignore incoming sights older than this, because they're stale.
     * This shouldn't happen if the camera is working well.
     */
    private final double m_maxSightAgeS;

    public final DoubleArrayLogger m_log_closestTarget;
    public final DoubleArrayLogger m_log_allTargets;
    public final DoubleArrayLogger m_log_coalescedTargets;

    private final TargetTranslator m_translator;
    /** state = f(takt seconds) from history. */
    private final DoubleFunction<StateSE2> m_history;
    /** Accumulation of targets we see; this is really for logging only. */
    private final TrailingHistory<Translation2d> m_allTargets;
    /** Coalesced targets */
    private final CoalescingCollection<Translation2d> m_targets;
    /** Side effect mutates targets. */
    private final SideEffect m_vision;
    private final IntLogger m_log_historySize;

    /** The closest target from the most recent update. */
    private Optional<Translation2d> m_closestTarget;

    public Targets(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            double maxSightAge,
            DoubleFunction<StateSE2> history) {
        m_reader = new CameraReader<>("objectVision", "targets",
                StructBuffer.create(Target.struct));
        LoggerFactory log = parent.type(this);
        m_log_age = log.doubleLogger(Level.TRACE, "target age");
        m_log_poseTimestamp = log.doubleLogger(Level.TRACE, "pose timestamp");
        m_maxSightAgeS = maxSightAge;
        m_log_historySize = log.intLogger(Level.TRACE, "history size");
        m_log_closestTarget = fieldLogger.doubleArrayLogger(Level.TRACE, "closest target");
        m_log_allTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "all targets");
        m_log_coalescedTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "coalesced targets");

        m_translator = new TargetTranslator(history);
        m_history = history;
        m_allTargets = new TrailingHistory<>();
        m_targets = new CoalescingCollection<>(
                new TrailingHistory<>(),
                new NearR2(NEARNESS_THRESHOLD),
                new CentroidR2());
        m_vision = Cache.ofSideEffect(this::update);
    }

    public void update() {
        List<TargetMeasurement> measurements = filteredRead();
        consumeMeasurements(measurements);
    }

    private List<TargetMeasurement> filteredRead() {
        // Read all the pending input.
        List<TargetMeasurement> measurements = read();
        List<TargetMeasurement> filteredMeasurements = new ArrayList<>();
        for (TargetMeasurement m : measurements) {
            m_log_poseTimestamp.log(() -> m.timestamp);
            double age = Takt.get() - m.timestamp;
            m_log_age.log(() -> age);
            if (age > m_maxSightAgeS) {
                if (DEBUG) {
                    System.out.printf("WARNING: ignoring stale sight %f\n", age);
                }
                continue;
            }
            filteredMeasurements.add(m);
        }
        return measurements;
    }

    private void consumeMeasurements(List<TargetMeasurement> measurements) {
        // Clean the history, relative to the current moment.
        // Previously, eviction only occurred when the robot could see something.
        double deadline = Takt.get() - HISTORY_DURATION;
        m_allTargets.evict(deadline);
        m_targets.evict(deadline);

        for (TargetMeasurement m : measurements) {
            m_allTargets.add(m.timestamp, m.location);
            m_targets.add(m.timestamp, m.location);
        }
        // Show the targets on the Field2d widget.

        // compute the closest target
        Pose2d robotPose = m_history.apply(Takt.get()).pose();
        m_closestTarget = ObjectPicker.closestObject(m_targets.getAll(), robotPose);

        // Show the closest target on the field2d widget.
        m_log_closestTarget.log(
                () -> m_closestTarget.stream().flatMapToDouble(
                        x1 -> DoubleStream.of(x1.getX(), x1.getY(), 0.0)).toArray());

        // Show coalesced targets on the field2d widget.
        m_log_coalescedTargets.log(
                () -> m_targets.getAll().stream().flatMapToDouble(
                        x2 -> DoubleStream.of(x2.getX(), x2.getY(), 0.0)).toArray());

        // Show *all* the targets.
        m_log_allTargets.log(
                () -> m_allTargets.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        m_log_historySize.log(() -> m_targets.size());
    }

    private List<TargetMeasurement> read() {
        List<CameraReader.Record<Target>> records = m_reader.getRecords();
        List<TargetMeasurement> measurements = new ArrayList<>();
        for (CameraReader.Record<Target> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
        return measurements;
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTargets() {
        // make sure the queue has been read if not already
        m_vision.run();
        return m_targets.getAll();
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTarget() {
        // make sure the queue has been read if not already
        m_vision.run();
        return m_closestTarget;
    }

}