package org.team100.lib.subsystems.discus.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusServo;
import org.team100.lib.visualization.ArmVisualization;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Gamepad;

public class SetupServo implements Runnable {
    private static final double OFFSET = Math.PI / 2;
    private static final double SCALE = 3;
    private final DiscusServo m_discus;
    private final ArmVisualization m_viz;

    public SetupServo() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        Gamepad controller = new Gamepad(0);

        m_discus = new DiscusServo(logger, currentLog);
        m_viz = new ArmVisualization(m_discus::getPosition, "discus", 0);
        m_discus.setDefaultCommand(m_discus.position(
                () -> OFFSET + SCALE * controller.getLeftX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // new Trigger(controller::getAButton).whileTrue(m_discus.home());
        // new Trigger(controller::getBButton).onTrue(m_discus.zero());
        new Trigger(controller::getWestFaceButton).whileTrue(m_discus.position(() -> 8));
        new Trigger(controller::getNorthFaceButton).whileTrue(m_discus.position(() -> -8));
        new Trigger(controller::getRightBumperButton).whileTrue(m_discus.position(() -> 12));
        new Trigger(controller::getLeftBumperButton).whileTrue(m_discus.position(() -> -12));
        new Trigger(controller::getSouthFaceButton).whileTrue(m_discus.position(() -> 20));
        new Trigger(controller::getEastFaceButton).whileTrue(m_discus.position(() -> -20));
    }

    @Override
    public void run() {
        m_viz.run();
    }
}
