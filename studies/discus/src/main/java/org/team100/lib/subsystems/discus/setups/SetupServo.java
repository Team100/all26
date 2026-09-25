package org.team100.lib.subsystems.discus.setups;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.discus.DiscusServo;
import org.team100.lib.visualization.ArmVisualization;

import edu.wpi.first.wpilibj.XboxController;

/** Adds profiled motion. */
public class SetupServo implements Runnable {
    private static final double OFFSET = Math.PI / 2;
    private static final double SCALE = 3;
    private final DiscusServo m_discus;
    private final ArmVisualization m_viz;

    public SetupServo(TotalCurrentLog currentLog) {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        XboxController controller = new XboxController(0);

        m_discus = new DiscusServo(logger, currentLog);
        m_viz = new ArmVisualization(m_discus::getPosition, "discus", 0);
        m_discus.setDefaultCommand(m_discus.position(
                () -> OFFSET + SCALE * controller.getLeftX()));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        // whileTrue(controller::getAButton, m_discus.home());
        // onTrue(controller::getBButton, m_discus.zero()
        whileTrue(controller::getXButton,
                m_discus.position(() -> 3));
        whileTrue(controller::getYButton,
                m_discus.position(() -> -8));
        whileTrue(controller::getRightBumperButton,
                m_discus.position(() -> 12));
        whileTrue(controller::getLeftBumperButton,
                m_discus.position(() -> -12));
        whileTrue(controller::getAButton,
                m_discus.position(() -> 20));
        whileTrue(controller::getBButton,
                m_discus.position(() -> -20));
    }

    @Override
    public void run() {
        m_viz.run();
    }
}
