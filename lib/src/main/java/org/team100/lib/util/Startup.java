package org.team100.lib.util;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiments;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.RobotController;
import org.wpilib.system.WPILibVersion;

/** Stuff we always do when the robot starts. */
public class Startup {
    public static void start() {
        // Print TEAM 100.
        Banner.printBanner();
        // Start the Redux event loop. This is really
        // only needed if you're doing setup.
        // CanandEventLoop.getInstance();
        // This is for setting up LaserCAN devices;
        // it's not needed unless you're doing setup.
        // CanBridge.runTCP();

        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());

        // Only works on RoboRIO 2.0.
        RobotController.setBrownoutVoltage(5.5);
        // Show the experiment picker on glass.
        Experiments.instance.show();
        // Show what the scheduler is doing.
        SmartDashboard.putData(CommandScheduler.getInstance());
        // Set the period to forever, to make the watchdog shut up.
        CommandScheduler.getInstance().setPeriod(100);
        // Make the joystick complainer shut up.
        DriverStationBackend.silenceJoystickConnectionWarning(true);
    }

}
