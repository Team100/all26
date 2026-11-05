package org.team100.lib.music.commands;

import java.util.List;

import org.team100.lib.music.MusicEvent;
import org.team100.lib.music.Player;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Move according to a schedule. */
public class MoveSequence extends Command {

    private final Timer m_timer;
    private final List<MusicEvent> m_events;
    private int m_nextEvent;
    Player player;

    public MoveSequence(Player player, List<MusicEvent> events) {
        this.player = player;
        m_events = events;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_nextEvent = 0;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_events.isEmpty())
            return;
        double currentTimeSec = m_timer.get();
        MusicEvent event = m_events.get(m_nextEvent);
        if (event.t() < currentTimeSec) {
            player.play(event.freq());
            m_nextEvent = m_nextEvent + 1;
        }
    }

    @Override
    public boolean isFinished() {
        return m_nextEvent == m_events.size();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
