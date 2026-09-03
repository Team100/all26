package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.subsystems.discus.setups.SetupServo;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

import org.team100.lib.subsystems.discus.setups.SetupBare;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {

    private final Runnable m_setup;

    public Robot() {
        /////////////////////////////
        //
        // Choose one of the setups.
        //

        // manual control
        m_setup = new SetupBare();
        
        // PID positional control
        //  m_setup = new SetupMech();

        // profiled control
        // m_setup = new SetupServo();
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_setup.run();
        // Show all the measurements at maximum rate.
        NetworkTableInstance.getDefault().flush();
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

}
