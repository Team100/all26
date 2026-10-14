package org.team100.lib.music;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

import javax.sound.midi.MetaMessage;
import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.Sequence;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

/**
 * Collates all the NOTE_ON and NOTE_OFF events into a single timeline.
 * 
 * Understands multiple tracks and multiple channels, but doesn't understand
 * chords within a single channel.
 */
public class EventIterator {
    private static final boolean DEBUG = false;

    /** (timestamp, (index, event)) */
    private TreeMap<Double, Map<Integer, MidiEvent>> m_events;
    private final double m_dt;

    public EventIterator(String filepath, double dt, double speed) {
        m_events = collate(filepath, speed);
        m_dt = dt;
    }

    /**
     * Find (index, event) from t-dt (exclusive) to t (inclusive).
     */
    public Map<Integer, MidiEvent> get(double t) {
        NavigableMap<Double, Map<Integer, MidiEvent>> m = m_events.subMap(t - m_dt, false, t, true);
        Map<Integer, MidiEvent> result = new TreeMap<>();
        // Walks the entries in time order, so the youngest is retained.
        for (Map<Integer, MidiEvent> r : m.values()) {
            result.putAll(r);
        }
        return result;
    }

    public double duration() {
        return m_events.lastKey();
    }

    private record Tempo(double time, double bpm) {
    }

    private static TreeMap<Double, Map<Integer, MidiEvent>> collate(String filepath, double speed) {
        // tempo events by tick
        TreeMap<Long, Tempo> tempos = new TreeMap<>();
        tempos.put(0l, new Tempo(0.0, 0.0));
        // note-on events by time by channel
        TreeMap<Double, Map<Integer, MidiEvent>> events = new TreeMap<>();
        Sequence seq = MidiUtil.sequence(filepath);
        if (seq.getDivisionType() > 1) {
            throw new IllegalArgumentException(String.format("%s division type is not PPQ", filepath));
        }
        // resolution is ticks per quarter note (PPQ)
        final int ticksPerBeat = seq.getResolution();
        Track[] tracks = seq.getTracks();
        List<Integer> trackChannelIndices = new ArrayList<>();
        for (int ti = 0; ti < tracks.length; ++ti) {
            Track t = tracks[ti];
            for (int ei = 0; ei < t.size(); ++ei) {
                MidiEvent e = t.get(ei);
                long tick = e.getTick();
                // the timestamp for this note depends on the entire
                // tempo history, not just the tempo at this time.
                Entry<Long, Tempo> tempo = tempos.floorEntry(tick);
                long dtick = tick - tempo.getKey();
                double sec = tempo.getValue().time
                        + MidiUtil.ticksToSec(dtick, tempo.getValue().bpm, ticksPerBeat);
                MidiMessage m = e.getMessage();
                if (m instanceof MetaMessage) {
                    MetaMessage mm = (MetaMessage) m;
                    byte[] data = mm.getData();
                    MetaMessageType mmt = MetaMessageType.get(mm.getType());
                    if (mmt == MetaMessageType.SET_TEMPO) {
                        int mpq = ((data[0] & 0xFF) << 16) |
                                ((data[1] & 0xFF) << 8) |
                                (data[2] & 0xFF);
                        double bpm = speed * 60000000.0 / mpq;
                        tempos.put(tick, new Tempo(sec, bpm));
                        if (DEBUG)
                            System.out.printf("tempo %.0f\n", bpm);
                    }
                    continue;
                }
                if (m instanceof ShortMessage) {
                    ShortMessage sm = (ShortMessage) m;
                    ShortMessageCommand smc = ShortMessageCommand.get(sm.getCommand());
                    if (smc == ShortMessageCommand.NOTE_OFF || smc == ShortMessageCommand.NOTE_ON) {
                        int key = sm.getData1();
                        int velocity = sm.getData2();
                        int channel = sm.getChannel();
                        // channels can be nonunique and nonsequential, so make a
                        // sequence.
                        int trackChannel = ti * 16 + channel;
                        int index = trackChannelIndices.indexOf(trackChannel);
                        if (index == -1) {
                            trackChannelIndices.add(trackChannel);
                            index = trackChannelIndices.size() - 1;
                        }
                        // We retain only one event per index per time slice, since we can take only
                        // one action per time slice.
                        // Because we're stepping through events in tick order, the youngest event will
                        // remain.
                        events.computeIfAbsent(sec, x -> new TreeMap<>()).put(index, e);
                        Note note = Note.get(key);
                        if (note == null) {
                            continue;
                        }
                        if (DEBUG)
                            System.out.printf("%f: Ch %d: %d: %s (%.0f) %d\n",
                                    sec, index, tick, note, note.hz, velocity);
                    }
                    continue;
                }
            }
        }
        System.out.printf("*** Loaded MIDI %s tracks: %d indices: %d\n",
                filepath, tracks.length, trackChannelIndices.size());

        return events;
    }

}
