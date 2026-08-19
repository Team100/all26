package org.team100.lib.logging;

import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.RobotController;

/** Robot-level logs, these used to pollute Robot.java. */
public class RobotLog {
    private final JvmLogger m_jvmLogger;
    private final DoubleLogger m_log_ds_MatchTime;
    private final BooleanLogger m_log_ds_AutonomousEnabled;
    private final BooleanLogger m_log_ds_TeleopEnabled;
    private final BooleanLogger m_log_ds_FMSAttached;
    private final DoubleLogger m_log_voltage;
    private final TotalCurrentLog m_totalCurrentLog;

    public RobotLog() {
        LoggerFactory logger = Logging.instance().rootLogger;
        LoggerFactory robotLogger = logger.name("Robot");
        m_jvmLogger = new JvmLogger(robotLogger);
        LoggerFactory dsLog = robotLogger.name("DriverStation");
        m_log_ds_MatchTime = dsLog.doubleLogger(Level.TRACE, "MatchTime");
        m_log_ds_AutonomousEnabled = dsLog.booleanLogger(Level.TRACE, "AutonomousEnabled");
        m_log_ds_TeleopEnabled = dsLog.booleanLogger(Level.TRACE, "TeleopEnabled");
        m_log_ds_FMSAttached = dsLog.booleanLogger(Level.TRACE, "FMSAttached");
        m_log_voltage = robotLogger.doubleLogger(Level.COMP, "voltage");
        m_totalCurrentLog = new TotalCurrentLog(Logging.instance().rootLogger);
    }

    public TotalCurrentLog totalCurrentLog() {
        return m_totalCurrentLog;
    }

    /** Logs robot-scope stuff, e.g. memory, voltage, current. */
    public void periodic() {
        m_jvmLogger.logGarbageCollectors();
        m_jvmLogger.logMemoryPools();
        m_jvmLogger.logMemoryUsage();
        m_log_ds_MatchTime.log(MatchState::getMatchTime);
        m_log_ds_AutonomousEnabled.log(RobotBase::isAutonomousEnabled);
        m_log_ds_TeleopEnabled.log(RobotBase::isTeleopEnabled);
        m_log_ds_FMSAttached.log(RobotState::isFMSAttached);
        m_log_voltage.log(RobotController::getBatteryVoltage);
        m_totalCurrentLog.log();
    }
}
