
package frc.robot;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.KrakenX44Motor;
import org.team100.lib.music.Player;
import org.team100.lib.music.Polyphony;
import org.team100.lib.music.TextPlayer;
import org.team100.lib.music.commands.Play;
import org.team100.lib.util.CanId;
import org.team100.lib.util.Startup;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class Robot extends TimedRobot {
    boolean MOTOR = true;
    private final Play play;

    public Robot() {
        Startup.start();
        final Player c0;
        final Player c1;
        final Player c2;
        final Player c3;
        if (MOTOR) {
            LoggerFactory log = Logging.root();
            TotalCurrentLog totalCurrentLog = new RobotLog(log).totalCurrentLog();
            Friction friction = new Friction(0, 0, 0, 0);
            NeutralMode100 neutral = NeutralMode100.COAST;
            MotorPhase phase = MotorPhase.FORWARD;
            CurrentLimit limit = new CurrentLimit(20, 20);
            PIDConstants pid = PIDConstants.zero();
            c0 = new KrakenX44Motor(
                    log, totalCurrentLog, new CanId(1), neutral,
                    phase, limit, friction, pid);
            c1 = new KrakenX44Motor(
                    log, totalCurrentLog, new CanId(2), neutral,
                    phase, limit, friction, pid);
            c2 = new KrakenX44Motor(
            log, totalCurrentLog, new CanId(3), neutral,
            phase, limit, friction, pid);
            c3 = new KrakenX44Motor(
            log, totalCurrentLog, new CanId(3), neutral,
            phase, limit, friction, pid);
        } else {
            c0 = new TextPlayer("c0");
            c1 = new TextPlayer("c1");
            c2 = new TextPlayer("c2");
            c3 = new TextPlayer(" c3");
        }
        // default speed for this tune is 120, which is fine.
        // play = new Play("popcorn2.mid", 1.0, new Polyphony(c0, c1));
        // play = new Play("take-the-a-train.mid", 1.0, new Polyphony(c0, c1, c2, c3));
        // play = new Play("waltz-no-2-trio.mid", 1.0, new Polyphony(c0, c1, c2));
        // play = new Play("quartet-ravel.mid", 1.0, new Polyphony(c0, c1, c2, c3));
        // jupter has ten tracks
        // play = new Play("jupiter.mid", 1.0, new Polyphony(c0, c1, c2, c3));
        // dvorak 9 has five tracks
        // play = new Play("dvorak-9.mid", 1.0, new Polyphony(c0, c1, c2, c3));
        play = new Play("dance-of-the-knights-prokofiev.mid", 1.0, new Polyphony(c0, c1, c2, c3));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(Commands.repeatingSequence(play));
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}
