package org.team100.lib.music.commands;

import java.util.Map;

import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;

import org.team100.lib.music.EventIterator;
import org.team100.lib.music.Note;
import org.team100.lib.music.Polyphony;
import org.team100.lib.music.ShortMessageCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Play extends Command {
    private static final boolean DEBUG = false;
    private EventIterator m_iter;
    private final Timer m_timer;
    private final Polyphony m_ensemble;

    public Play(String filepath, double speed, Polyphony ensemble) {
        m_iter = new EventIterator(filepath, 0.02, speed);
        m_timer = new Timer();
        m_ensemble = ensemble;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double nowSec = m_timer.get();
        Map<Integer, MidiEvent> events = m_iter.get(nowSec);
        if (events == null) {
            return;
        }
        // (index, event)
        for (Map.Entry<Integer, MidiEvent> entry : events.entrySet()) {
            MidiEvent e = entry.getValue();
            long tick = e.getTick();
            MidiMessage m = e.getMessage();
            if (m instanceof ShortMessage) {
                ShortMessage sm = (ShortMessage) m;
                ShortMessageCommand smc = ShortMessageCommand.get(sm.getCommand());
                int velocity = sm.getData2();
                // don't use the channel in the message, use the index.
                int channel = entry.getKey();
                if (smc == ShortMessageCommand.NOTE_ON && velocity > 0) {
                    int key = sm.getData1();
                    Note note = Note.get(key);
                    if (note == null) {
                        continue;
                    }
                    if (DEBUG)
                        System.out.printf(
                                "t %.2f, ch %d, tick %d, note %s (%.0f)\n",
                                nowSec, channel, tick, note, note.hz);
                    m_ensemble.play(channel, note.hz);
                } else {
                    if (DEBUG)
                        System.out.printf(
                                "t %.2f, ch %d, tick %d, release\n",
                                nowSec, channel, tick);
                    m_ensemble.play(channel, 0);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > m_iter.duration();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
