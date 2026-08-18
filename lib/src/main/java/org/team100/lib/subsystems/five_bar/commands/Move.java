package org.team100.lib.subsystems.five_bar.commands;

import java.util.Optional;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.five_bar.FiveBarCartesian;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move in a straight line via interpolation.
 * 
 * This is an example of a command which is too complicated to inline into the
 * subsystem itself, because it has state (the current location, progress
 * towards the endpoint, etc).
 */
public class Move extends Command {

    private final FiveBarCartesian m_fiveBar;
    private final Translation2d m_goal;
    private final ProfileR1 m_profile;
    private final Translation2dLogger m_log_start;
    private final Translation2dLogger m_log_goal;
    private final DoubleLogger m_log_s;

    private ControlR1 m_setpoint;
    private ModelR1 m_profileGoal;

    private Translation2d m_start;
    private double m_distance;
    private boolean m_done;

    public Move(
            LoggerFactory parent,
            FiveBarCartesian fiveBar,
            Translation2d goal,
            double velocity) {
        LoggerFactory log = parent.type(this);
        m_fiveBar = fiveBar;
        m_goal = goal;
        m_profile = new WPITrapezoidProfileR1(velocity, 1);
        m_log_start = log.translation2dLogger(Level.COMP, "start");
        m_log_goal = log.translation2dLogger(Level.COMP, "goal");
        m_log_s = log.doubleLogger(Level.COMP, "s");
        addRequirements(fiveBar);
    }

    @Override
    public void initialize() {
        Optional<Translation2d> optStart = m_fiveBar.getPosition();
        if (optStart.isEmpty()) {
            m_done = true;
            System.out.println("Move: infeasible start");
            return;
        }
        m_start = optStart.get();
        m_log_start.log(() -> m_start);
        m_log_goal.log(() -> m_goal);
        m_distance = m_start.getDistance(m_goal);
        m_setpoint = new ControlR1();
        m_profileGoal = new ModelR1(m_distance, 0);
        m_done = false;
    }

    @Override
    public void execute() {
        if (m_done)
            return;
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_profileGoal);
        ControlR1 c = m_setpoint;
        double s = c.x() / m_distance;
        m_log_s.log(() -> s);
        Translation2d setpoint = m_start.interpolate(m_goal, s);
        double togo = setpoint.getDistance(m_goal);
        if (togo < 0.001) {
            m_fiveBar.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_fiveBar.setPosition(setpoint);
    }

    /**
     * Using our own "done" method instead of the Command "isFinished" method makes
     * it easier to compose.
     */
    public boolean done() {
        return m_done;
    }

}
