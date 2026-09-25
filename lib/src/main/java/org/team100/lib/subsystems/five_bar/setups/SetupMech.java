package org.team100.lib.subsystems.five_bar.setups;

import static org.team100.lib.util.TriggerUtil.onTrue;
import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.kinematics.five_bar.Scenario;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.five_bar.FiveBarMech;
import org.team100.lib.subsystems.five_bar.Pen;
import org.team100.lib.visualization.FiveBarVisualization;

import edu.wpi.first.wpilibj.XboxController;

public class SetupMech implements Runnable {
    private final double CONTROL_SCALE = 0.1;
    private final FiveBarMech m_fiveBar;
    private final Pen m_pen;
    private final FiveBarVisualization m_viz;

    public SetupMech(Scenario scenario) {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        XboxController controller = new XboxController(0);

        m_fiveBar = new FiveBarMech(logger, currentLog, scenario);
        m_pen = new Pen();
        m_viz = new FiveBarVisualization(scenario, m_fiveBar::getJointPositions);
        m_fiveBar.setDefaultCommand(m_fiveBar.position(
                () -> CONTROL_SCALE * controller.getLeftX(), // axis 0, "a" and "d" in the sim
                () -> CONTROL_SCALE * controller.getRightX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // button 1, "z" in the sim
        whileTrue(controller::getAButton,
                m_fiveBar.home());
        // button 2, "x" in the sim
        onTrue(controller::getBButton,
                m_fiveBar.zero());
        // button 3, "c" in the sim
        onTrue(controller::getXButton,
                m_pen.down());
        // button 4, "v" in the sim.
        onTrue(controller::getYButton,
                m_pen.up());
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
