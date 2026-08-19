package org.team100.lib.subsystems.five_bar.setups;

import org.team100.lib.kinematics.five_bar.Scenario;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.five_bar.FiveBarMech;
import org.team100.lib.subsystems.five_bar.Pen;
import org.team100.lib.visualization.FiveBarVisualization;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.command2.button.Trigger;

public class SetupMech implements Runnable {
    private final double CONTROL_SCALE = 0.1;
    private final FiveBarMech m_fiveBar;
    private final Pen m_pen;
    private final FiveBarVisualization m_viz;

    public SetupMech(Scenario scenario) {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        Gamepad controller = new Gamepad(0);

        m_fiveBar = new FiveBarMech(logger, currentLog, scenario);
        m_pen = new Pen();
        m_viz = new FiveBarVisualization(scenario, m_fiveBar::getJointPositions);
        m_fiveBar.setDefaultCommand(m_fiveBar.position(
                () -> CONTROL_SCALE * controller.getLeftX(), // axis 0, "a" and "d" in the sim
                () -> CONTROL_SCALE * controller.getRightX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // button 1, "z" in the sim
        new Trigger(controller::getSouthFaceButton).whileTrue(m_fiveBar.home());
        // button 2, "x" in the sim
        new Trigger(controller::getEastFaceButton).onTrue(m_fiveBar.zero());
        // button 3, "c" in the sim
        new Trigger(controller::getWestFaceButton).onTrue(m_pen.down());
        // button 4, "v" in the sim.
        new Trigger(controller::getNorthFaceButton).onTrue(m_pen.up());
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
