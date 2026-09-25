package first.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dynamics.p.PDynamics;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.reference.r1.ProfileReferenceR1;
import org.team100.lib.reference.r1.ReferenceR1;
import org.team100.lib.subsystems.r1.SingleLinearSubsystem;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Startup;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot100 {
    private final RobotLog m_robotLog;
    private final DriverXboxControl m_control;
    private final SingleLinearSubsystem m_subsystem;

    public Robot() {
        Startup.start();
        LoggerFactory log = Logging.instance().rootLogger;
        m_robotLog = new RobotLog(log);
        TotalCurrentLog currentLog = m_robotLog.totalCurrentLog();

        // CONFIGURATION

        CanId canId1 = new CanId(1);
        NeutralMode100 neutral = NeutralMode100.COAST;
        MotorPhase phase1 = MotorPhase.FORWARD;
        CurrentLimit limit = new CurrentLimit(1, 1);
        double gearRatio = 6.0;
        double wheelDiameterM = 0.025;
        Friction friction = new Friction(0.32, 0.32, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makePositionPID(1);
        PDynamics dynamics = new PDynamics(1);
        double xtolerance = 0.01;
        double vtolerance = 0.01;
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(0.1, 0.25, xtolerance);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, xtolerance, vtolerance);

        // SUBSYSTEM

        m_subsystem = new SingleLinearSubsystem(
                log, currentLog, canId1, neutral, phase1, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref, xtolerance, vtolerance, true);

        // BINDINGS

        m_subsystem.setDefaultCommand(m_subsystem.stop().withName("stop"));
        m_control = new DriverXboxControl(log, 0);
        whileTrue(m_control::start, m_subsystem.zero().withName("zero"));
        whileTrue(m_control::a, m_subsystem.position(0).withName("in"));
        whileTrue(m_control::b, m_subsystem.position(0.1).withName("out"));
        whileTrue(m_control::leftBumper, m_subsystem.velocity(-0.1).withName("negative"));
        whileTrue(m_control::rightBumper, m_subsystem.velocity(0.1).withName("positive"));
        whileTrue(m_control::back, m_subsystem.voltage(0.5).withName("voltage"));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        // Poll for logs after all the actuation is done
        LogPoller.log();
        if (Experiments.INSTANCE.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void close() {
        super.close();
        m_subsystem.close();
    }

}