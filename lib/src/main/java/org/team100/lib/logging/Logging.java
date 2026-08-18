package org.team100.lib.logging;

import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.util.NamedChooser;

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
    private static final Logging instance = new Logging();

    // Required because SmartDashboard keeps only a weak reference.
    private final SendableChooser<Level> m_LevelChooser;
    private final PrimitiveLogger ntLogger;
    /** Root is "field", with .type = Field2d as required by glass. */
    public final LoggerFactory fieldLogger;
    /** Root is "log". */
    public final LoggerFactory rootLogger;
    /** Saves getSelected() calls */
    private Level m_selectedLevel;

    /**
     * Clients should use the static instance, not the constructor.
     */
    private Logging() {
        m_LevelChooser = new NamedChooser<>("Log Level");
        for (Level level : Level.values()) {
            m_LevelChooser.addOption(level.name(), level);
        }
        m_LevelChooser.setDefaultOption(DEFAULT_LEVEL.name(), DEFAULT_LEVEL);
        m_selectedLevel = DEFAULT_LEVEL;
        ntLogger = new NTPrimitiveLogger();
        fieldLogger = new LoggerFactory(this::getLevel, "field", ntLogger);
        fieldLogger.stringLogger(Level.COMP, ".type").log(() -> "Field2d");
        rootLogger = new LoggerFactory(this::getLevel, "log", ntLogger);
        // turn off the CTRE log we never use
        SignalLogger.enableAutoLogging(false);
        SmartDashboard.putData(m_LevelChooser);
        m_LevelChooser.onChange(this::update);
    }

    public void update(Level level) {
        m_selectedLevel = level;
    }

    public int keyCount() {
        if (ntLogger != null)
            return ntLogger.keyCount();
        return 0;
    }

    public Level getLevel() {
        return m_selectedLevel;
    }

    /** The logging singleton. */
    public static Logging instance() {
        return instance;
    }
}