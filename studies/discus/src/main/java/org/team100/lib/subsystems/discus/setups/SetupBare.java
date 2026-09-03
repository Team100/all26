package org.team100.lib.subsystems.discus.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusBare;
import org.team100.lib.visualization.ArmVisualization;
import org.wpilib.driverstation.Gamepad;

import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        // controller::getLeftX));
        m_discus.setDefaultCommand(m_discus.voltage(
                () -> 4 * controller.getLeftX()));
        new Trigger(controller::getAButton).whileTrue(
                m_discus.voltage(() -> 8));
        new Trigger(controller::getBButton).whileTrue(
                m_discus.voltage(() -> 12));

        // WARNING: constant current produces constant acceleration
        // up to MAX SPEED! Don't run this for too long, and be careful
        // with high current values.
        new Trigger(controller::getXButton).whileTrue(
                m_discus.current(() -> 1).withTimeout(1));

    }

    @Override
    public void run() {
        m_viz.run();
    }
}
