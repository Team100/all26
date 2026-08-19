package org.team100.lib.subsystems.discus.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusBare;
import org.team100.lib.visualization.ArmVisualization;
import org.wpilib.driverstation.Gamepad;

public class SetupBare implements Runnable {
    private final DiscusBare m_discus;
    private final ArmVisualization m_viz;

    public SetupBare() {
        Logging logging = Logging.instance();
        LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        Gamepad controller = new Gamepad(0);
        m_discus = new DiscusBare(logger, currentLog);
        m_viz = new ArmVisualization(m_discus::getPosition, "discus", 0);
        // m_discus.setDefaultCommand(m_discus.dutyCycle(
        //         controller::getLeftX));
        m_discus.setDefaultCommand(m_discus.voltage(
                controller::getLeftX));
    }

    @Override
    public void run() {
        m_viz.run();
    }
}
