package org.team100.lib.subsystems.five_bar.setups;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import org.team100.lib.kinematics.five_bar.Scenario;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.five_bar.FiveBarCartesian;
import org.team100.lib.visualization.FiveBarVisualization;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SetupCartesian implements Runnable {
    private static final double XMAX = 0.15;
    private static final double YMAX = 0.1;
    private final FiveBarCartesian m_fiveBar;
    private final FiveBarVisualization m_viz;

    public SetupCartesian(Scenario scenario) {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        XboxController controller = new XboxController(0);

        m_fiveBar = new FiveBarCartesian(logger, currentLog, scenario);
        m_viz = new FiveBarVisualization(scenario, m_fiveBar::getJointPositions);
        m_fiveBar.setDefaultCommand(m_fiveBar.position(
                () -> new Translation2d(
                        // x-right is positive, but here we want x-left to be positive.
                        -1.0 * XMAX * controller.getRightX(), // axis 4
                        // y-down is positive, which is correct
                        YMAX * controller.getRightY()))); // axis 5

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        new Trigger(controller::getAButton).whileTrue(m_fiveBar.home());
        new Trigger(controller::getBButton).onTrue(m_fiveBar.zero());

        // Make a little square. Also illustrates "print" commands for
        // debugging.
        double delay = 0.5;
        // a little square.
        double XX = 0.05;
        double YY = 0.05;
        new Trigger(controller::getXButton).whileTrue(
                Commands.sequence(
                        print("move to origin"),
                        m_fiveBar.move(new Translation2d(0, 0)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move to lower left"),
                        m_fiveBar.move(new Translation2d(XX, YY)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move to lower right"),
                        m_fiveBar.move(new Translation2d(-XX, YY)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move to upper right"),
                        m_fiveBar.move(new Translation2d(-XX, -YY)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move to upper left"),
                        m_fiveBar.move(new Translation2d(XX, -YY)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move to lower left"),
                        m_fiveBar.move(new Translation2d(XX, YY)),
                        print("wait"),
                        Commands.waitSeconds(delay),
                        print("move back home"),
                        m_fiveBar.move(new Translation2d(0, 0))));
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
