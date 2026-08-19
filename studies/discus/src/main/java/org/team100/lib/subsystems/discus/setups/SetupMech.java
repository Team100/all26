package org.team100.lib.subsystems.discus.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusMech;
import org.team100.lib.visualization.ArmVisualization;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.Gamepad;

public class SetupMech implements Runnable {
    private final double CONTROL_SCALE = 1;
    private final DiscusMech m_discus;
    private final ArmVisualization m_viz;

    public SetupMech() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        Gamepad controller = new Gamepad(0);

        m_discus = new DiscusMech(logger, currentLog);
        m_viz = new ArmVisualization(m_discus::getPosition, "discus", 0);
        // m_discus.setDefaultCommand(m_discus.position(
        // () -> CONTROL_SCALE * controller.getLeftX()));
        m_discus.setDefaultCommand(m_discus.velocity(
                () -> CONTROL_SCALE * controller.getLeftX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // button 1, "z" in the sim
        new Trigger(controller::getSouthFaceButton).whileTrue(m_discus.home());
        // button 2, "x" in the sim
        new Trigger(controller::getEastFaceButton).whileTrue(m_discus.zero());
        new Trigger(controller::getWestFaceButton).whileTrue(m_discus.position(() -> 2));
        new Trigger(controller::getNorthFaceButton).whileTrue(m_discus.position(() -> -2));

        // set voltage directly to tune friction.
        new Trigger(controller::getLeftBumperButton)
                .whileTrue(m_discus.friction(() -> 0.2 * controller.getRightX()));
    }

    @Override
    public void run() {
        m_viz.run();
    }
}
