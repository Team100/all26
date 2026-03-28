package org.team100.lib.logging;

import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.util.NamedChooser;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logging singleton.
 * 
 * If you use this logger you'll want to set the log level.
 */
public class Logging {
    private static final Level DEFAULT_LEVEL = Level.DEBUG;

    private final PrimitiveLogger ntLogger;
    private volatile Level m_cachedLevel = DEFAULT_LEVEL;
    private volatile boolean m_fmsLatched;
    /**
     * Precomputed allow results, indexed by Level.ordinal(). Replaced (not
     * mutated) via copy-on-write for thread visibility. During competition
     * the level is frozen, so this reference never changes.
     */
    private volatile boolean[] m_allowed = computeAllowed(DEFAULT_LEVEL);

    private static final SendableChooser<Level> m_LevelChooser = new NamedChooser<>("Log Level");

    static {
        for (Level level : Level.values()) {
            m_LevelChooser.addOption(level.name(), level);
        }
        m_LevelChooser.setDefaultOption(DEFAULT_LEVEL.name(), DEFAULT_LEVEL);
        SmartDashboard.putData(m_LevelChooser);
    }

    private static final Logging instance = new Logging();

    /**
     * root is "field", with a ".type"->"Field2d" entry as required by glass.
     */
    public final LoggerFactory fieldLogger;
    /** root is "log". */
    public final LoggerFactory rootLogger;

    /**
     * Clients should use the static instance, not the constructor.
     */
    private Logging() {
        ntLogger = new NTPrimitiveLogger();
        fieldLogger = new LoggerFactory(this::isAllowed, "field", ntLogger);
        rootLogger = new LoggerFactory(this::isAllowed, "log", ntLogger);
        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");

        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);

        refreshLogLevel();
    }

    public int keyCount() {
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    /**
     * Called every robot loop iteration to pick up dashboard level changes.
     * If FMS is attached, latches to COMP for the remainder of the session
     * and subsequent calls are a single volatile read (no chooser lock).
     * Otherwise reads the dashboard chooser and rebuilds the allow cache
     * only when the selection has changed.
     */
    public void refreshLogLevel() {
        if (m_fmsLatched) {
            return;
        }
        if (DriverStation.isFMSAttached()) {
            m_fmsLatched = true;
            m_cachedLevel = Level.COMP;
            m_allowed = computeAllowed(Level.COMP);
            return;
        }
        Level selected = m_LevelChooser.getSelected();
        if (selected != null && selected != m_cachedLevel) {
            m_cachedLevel = selected;
            m_allowed = computeAllowed(selected);
        }
    }

    public Level getLevel() {
        return m_cachedLevel;
    }

    public boolean isAllowed(Level level) {
        return m_allowed[level.ordinal()];
    }

    private static boolean[] computeAllowed(Level current) {
        boolean[] allowed = new boolean[Level.values().length];
        for (Level l : Level.values()) {
            allowed[l.ordinal()] = current.admit(l);
        }
        return allowed;
    }

    /** For testing only. Resets the FMS latch and cached level to defaults. */
    void resetForTest() {
        m_fmsLatched = false;
        m_cachedLevel = DEFAULT_LEVEL;
        m_allowed = computeAllowed(DEFAULT_LEVEL);
    }

    /** The logging singleton. */
    public static Logging instance() {
        return instance;
    }
}