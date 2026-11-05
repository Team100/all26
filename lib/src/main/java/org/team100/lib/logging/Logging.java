package org.team100.lib.logging;

import org.team100.lib.logging.primitive.NTPrimitiveLogger;
import org.team100.lib.logging.primitive.PrimitiveLogger;
import org.team100.lib.util.NamedChooser;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;

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
    private final LoggerFactory fieldLogger;
    /** Root is "log". */
    private final LoggerFactory rootLogger;
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
        // Turn off the CTRE log we never use.
        SignalLogger.enableAutoLogging(false);
        // Disable the REV log we never use.
        StatusLogger.disableAutoLogging();
        SmartDashboard.putData(m_LevelChooser);
        m_LevelChooser.onChange(this::update);
    }

    public static LoggerFactory root() {
        return instance.rootLogger;
    }

    public static LoggerFactory field() {
        return instance.fieldLogger;
    }

    public static boolean admit(Level level) {
        return instance.getLevel().admit(level);
    }

    public static int keyCount() {
        if (instance.ntLogger != null)
            return instance.ntLogger.keyCount();
        return 0;
    }

    private void update(Level level) {
        m_selectedLevel = level;
    }

    private Level getLevel() {
        return m_selectedLevel;
    }
}