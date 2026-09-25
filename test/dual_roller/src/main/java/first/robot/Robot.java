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
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.subsystems.r1.DualRollerSubsystem;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Startup;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot100 {
    private final RobotLog m_robotLog;
    private final DriverXboxControl m_control;
    private final DualRollerSubsystem m_subsystem;

    public Robot() {
        Startup.start();
        LoggerFactory log = Logging.instance().rootLogger;
        m_robotLog = new RobotLog(log);
        TotalCurrentLog currentLog = m_robotLog.totalCurrentLog();

        // CONFIGURATION

        CanId canId1 = new CanId(1);
        CanId canId2 = new CanId(2);
        NeutralMode100 neutral = NeutralMode100.COAST;
        MotorPhase phase1 = MotorPhase.FORWARD;
        MotorPhase phase2 = MotorPhase.REVERSE;
        CurrentLimit limit = new CurrentLimit(1, 1);
        double gearRatio = 6.0;
        double wheelDiameterM = 0.025;
        Friction friction = new Friction(0.32, 0.32, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makePositionPID(1);
        PDynamics dynamics = new PDynamics(1);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(0.25);
        double tolerance = 0.05;
        VelocityReferenceR1 ref = new VelocityProfileReferenceR1(
                log, () -> profile, tolerance);

        // SUBSYSTEM

        m_subsystem = new DualRollerSubsystem(
                log, currentLog, canId1, canId2, neutral, phase1, phase2, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref, 0.01, true);

        // BINDINGS

        m_subsystem.setDefaultCommand(m_subsystem.stop().withName("stop"));
        m_control = new DriverXboxControl(log, 0);
        whileTrue(m_control::leftBumper, m_subsystem.velocity(-0.1).withName("negative"));
        whileTrue(m_control::rightBumper, m_subsystem.velocity(0.1).withName("positive"));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotLog.periodic();
        m_control.periodic();
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