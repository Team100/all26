package org.team100.lib.targeting;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.ObjectCache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A Simulated target designator that can choose one of two targets.
 * 
 * It provides Field2d visualization of the target using the name "target".
 */
public class TargetDesignator {
    /** Approximately the center of the field */
    public static final Translation2d A = new Translation2d(8, 4);
    /** Some other target. */
    public static final Translation2d B = new Translation2d(4, 2);
    private final DoubleArrayLogger m_log_field_target;
    private final ObjectCache<Translation2d> m_targetCache;
    /** Used only by update(). */
    private Translation2d m_target;

    public TargetDesignator(LoggerFactory fieldLogger, Translation2d initial) {
        m_log_field_target = fieldLogger.doubleArrayLogger(Level.COMP, "target");
        m_target = initial;
        m_targetCache = Cache.of(this::update);
    }

    public Translation2d getTarget() {
        return m_targetCache.get();
    }

    public Command a() {
        return Commands.runOnce(() -> set(A));
    }

    public Command b() {
        return Commands.runOnce(() -> set(B));
    }

    public void periodic() {
        m_log_field_target.log(this::poseArray);
    }

    private double[] poseArray() {
        Translation2d t = m_targetCache.get();
        return new double[] { t.getX(), t.getY(), 0 };
    }

    private Translation2d update() {
        return m_target;
    }

    private void set(Translation2d t) {
        m_target = t;
    }

}
