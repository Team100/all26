package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

import org.team100.lib.camera.Camera;
import org.team100.lib.camera.Offset;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.coherence.Takt;
import org.team100.lib.geometry.r2.CentroidR2;
import org.team100.lib.geometry.r2.NearR2;
import org.team100.lib.localization.StateSampler;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.network.CameraReader;
import org.team100.lib.util.CoalescingCollection;
import org.team100.lib.util.TrailingHistory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.StructBuffer;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 */
public class Targets extends CameraReader<Target> {
    private static final boolean DEBUG = false;

    /** Ignore sightings farther away than this. */
    private static final double MAX_DISTANCE = 4.0;
    /** Forget sights older than this. */
    private static final double HISTORY_DURATION = 1.0;
    /** Targets closer than this to each other are combined */
    private static final double NEARNESS_THRESHOLD = 0.15;

    /**
     * Ignore incoming sights older than this, because they're stale.
     * This shouldn't happen if the camera is working well.
     */
    private final double m_maxSightAgeS;

    public final DoubleArrayLogger m_log_closestTarget;
    public final DoubleArrayLogger m_log_allTargets;
    public final DoubleArrayLogger m_log_coalescedTargets;

    /** state = f(takt seconds) from history. */
    private final StateSampler m_history;
    /** Accumulation of targets we see; this is really for logging only. */
    private final TrailingHistory<Translation2d> m_allTargets;
    /** Coalesced targets */
    private final CoalescingCollection<Translation2d> m_targets;
    /** Side effect mutates targets. */
    private final SideEffect m_vision;
    private final IntLogger m_log_historySize;
    private final DoubleLogger m_log_age;
    private final DoubleLogger m_log_poseTimestamp;

    /** The closest target from the most recent update. */
    private Optional<Translation2d> m_closestTarget;

    public Targets(
            LoggerFactory parent,
            LoggerFactory fieldLogger,
            double maxSightAge,
            StateSampler history) {
        super(parent, "objectVision", "targets", StructBuffer.create(Target.struct));
        LoggerFactory log = parent.type(this);
        m_maxSightAgeS = maxSightAge;
        m_log_historySize = log.intLogger(Level.TRACE, "history size");
        m_log_closestTarget = fieldLogger.doubleArrayLogger(Level.TRACE, "closest target");
        m_log_allTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "all targets");
        m_log_coalescedTargets = fieldLogger.doubleArrayLogger(Level.TRACE, "coalesced targets");
        m_log_age = log.doubleLogger(Level.TRACE, "target age");
        m_log_poseTimestamp = log.doubleLogger(Level.TRACE, "pose timestamp");
        m_history = history;
        m_allTargets = new TrailingHistory<>();
        m_targets = new CoalescingCollection<>(
                new TrailingHistory<>(),
                new NearR2(NEARNESS_THRESHOLD),
                new CentroidR2());
        m_vision = Cache.ofSideEffect(this::update);
    }

    /**
     * Clean the history, relative to the current moment.
     * 
     * Previously, eviction only occurred when the robot could see something.
     */
    @Override
    protected void beginUpdate() {
        double deadline = Takt.get() - HISTORY_DURATION;
        m_allTargets.evict(deadline);
        m_targets.evict(deadline);
    }

    /**
     * Transform sightings into field-relative targets.
     */
    @Override
    protected void perValue(Camera camera, Target[] sights) {
        for (Target sight : sights) {
            // server timestamp in sec
            double timeSec = (double) sight.getTimestamp() / 1e6;

            double age = Takt.get() - timeSec;
            m_log_age.log(() -> age);

            if (age > m_maxSightAgeS) {
                if (DEBUG) {
                    System.out.printf("WARNING: ignoring stale sight %f\n", age);
                }
                continue;
            }

            m_log_poseTimestamp.log(() -> timeSec);
            Pose2d robotPose = m_history.get(timeSec).pose();
            Transform3d cameraOffset = Offset.get(camera).offset();
            Optional<Translation2d> ot = TargetLocalizer.cameraRotToFieldRelative(
                    robotPose,
                    cameraOffset,
                    sight.sight());
            if (ot.isEmpty())
                continue;
            Translation2d t = ot.get();
            double distance = t.getDistance(robotPose.getTranslation());
            if (distance > MAX_DISTANCE) {
                if (DEBUG)
                    System.out.println("Target is too far away.");
                continue;
            }
            m_allTargets.add(timeSec, t);
            m_targets.add(timeSec, t);
        }
    }

    /**
     * Show the targets on the Field2d widget.
     */
    @Override
    protected void finishUpdate() {
        // compute the closest target
        Pose2d robotPose = m_history.get(Takt.get()).pose();

        m_closestTarget = ObjectPicker.closestObject(m_targets.getAll(), robotPose);

        // Show the closest target on the field2d widget.
        m_log_closestTarget.log(
                () -> m_closestTarget.stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        // Show coalesced targets on the field2d widget.
        m_log_coalescedTargets.log(
                () -> m_targets.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        // Show *all* the targets.
        m_log_allTargets.log(
                () -> m_allTargets.getAll().stream().flatMapToDouble(
                        x -> DoubleStream.of(x.getX(), x.getY(), 0.0)).toArray());

        m_log_historySize.log(() -> m_targets.size());
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