package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LogPoller;
import org.team100.lib.network.Sync;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final Sync sync;

    public Robot() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        sync = new Sync(inst);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        sync.run();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        // Poll for logs after all the actuation is done
        LogPoller.log();
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
