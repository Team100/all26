package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.network.Sync;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.NetworkTableInstance;

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
