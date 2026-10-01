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
import org.team100.lib.motor.ctre.CTREStartup;
import org.team100.lib.profile.r1.AccelLimitedVelocityProfileR1;
import org.team100.lib.profile.r1.VelocityProfileR1;
import org.team100.lib.reference.r1.VelocityProfileReferenceR1;
import org.team100.lib.reference.r1.VelocityReferenceR1;
import org.team100.lib.subsystems.r1.DualRollerSubsystem;
import org.team100.lib.util.CanBusId;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Startup;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.networktables.NetworkTableInstance;

public class Robot extends TimedRobot100 {
    private final RobotLog m_robotLog;
    private final DriverXboxControl m_control;

    private final DualRollerSubsystem m_subsystem_drum_top;
    private final DualRollerSubsystem m_subsystem_drum_bottom;
    private final DualRollerSubsystem m_subsystem_feeder;

    public Robot() {
        Startup.start();
        CTREStartup.start();
        LoggerFactory log = Logging.instance().rootLogger;
        m_robotLog = new RobotLog(log);
        TotalCurrentLog currentLog = m_robotLog.totalCurrentLog();

        // CONFIGURATION

        CanId canId13 = new CanId(13);
        CanId canId14 = new CanId(14);
        CanId canId15 = new CanId(15);
        CanId canId16 = new CanId(16);
        CanId canId17 = new CanId(17);
        CanId canId18 = new CanId(18);
        CanBusId busId = new CanBusId(0);
        NeutralMode100 neutral = NeutralMode100.COAST;
        MotorPhase phase1 = MotorPhase.FORWARD;
        MotorPhase phase2 = MotorPhase.REVERSE;
        CurrentLimit limit = new CurrentLimit(30, 30);
        double gearRatio = 6.0;
        double wheelDiameterM = 0.025;
        Friction friction = new Friction(0.32, 0.32, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makeVelocityPID(0.03);
        PDynamics dynamics = new PDynamics(1);
        VelocityProfileR1 profile = new AccelLimitedVelocityProfileR1(10);
        double tolerance = 0.05;
        VelocityReferenceR1 ref_drum = new VelocityProfileReferenceR1(
                log, () -> profile, tolerance);
        VelocityReferenceR1 ref_feeder = new VelocityProfileReferenceR1(
                log, () -> profile, tolerance);

        // SUBSYSTEMs

        m_subsystem_drum_top = new DualRollerSubsystem(
                log.name("drum top"), currentLog, canId13, canId15, busId, neutral, phase1, phase2, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref_drum, 0.01, true);

        m_subsystem_drum_bottom = new DualRollerSubsystem(
                log.name("drum bottom"), currentLog, canId14, canId16, busId, neutral, phase1, phase2, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref_drum, 0.01, true);

        m_subsystem_feeder = new DualRollerSubsystem(
                log.name("feeder"), currentLog, canId17, canId18, busId, neutral, phase1, phase2, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref_feeder, 0.01, true);

        // BINDINGS

        m_subsystem_drum_top.setDefaultCommand(m_subsystem_drum_top.stop().withName("stop"));
        m_control = new DriverXboxControl(log, 0);
        whileTrue(m_control::a,
                Commands.parallel(
                        m_subsystem_drum_top.velocity(-0.75),
                        m_subsystem_drum_bottom.velocity(-0.75)).withName("drum negative"));
        whileTrue(m_control::b,
                Commands.parallel(
                        m_subsystem_drum_top.velocity(0.75),
                        m_subsystem_drum_bottom.velocity(0.75)).withName("drum positive"));
        // whileTrue(m_control::back, m_subsystem.voltage(0.5).withName("voltage"));

        m_subsystem_drum_bottom.setDefaultCommand(m_subsystem_drum_bottom.stop().withName("stop"));
        // whileTrue(m_control::leftTrigger,
        // m_subsystem2.velocity(-1).withName("negative"));
        // whileTrue(m_control::rightTrigger,
        // m_subsystem2.velocity(1).withName("positive"));
        // whileTrue(m_control2::a, m_subsystem.voltage(0.5).withName("voltage"));

        m_subsystem_feeder.setDefaultCommand(m_subsystem_feeder.stop().withName("stop"));
        whileTrue(m_control::x, m_subsystem_feeder.velocity(-0.75).withName("feeder negative"));
        whileTrue(m_control::y, m_subsystem_feeder.velocity(0.75).withName("feeder positive"));
        // whileTrue(m_control3::b, m_subsystem.voltage(0.5).withName("voltage"));
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
        m_subsystem_drum_top.close();
        m_subsystem_drum_bottom.close();
        m_subsystem_feeder.close();
    }

}