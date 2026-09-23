package frc.robot;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;

/**
 * This should contain all the hardware of the robot: all the subsystems etc
 * that the Binder and Auton classes may want to use.
 */

public class Machinery {
    private static final LoggerFactory logger = Logging.instance().rootLogger;
    public final Climber m_Climber;
    public final ClimberExtension m_ClimberExtension;

    public Machinery() {
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        m_ClimberExtension = new ClimberExtension(logger, currentLog);
        m_Climber = new Climber(logger, currentLog);
    }

    public void periodic() {
    }

    public void close() {
    }

}
