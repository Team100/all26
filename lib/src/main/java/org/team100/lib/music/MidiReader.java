package org.team100.lib.music;

import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

/**
 * Read a MIDI file.
 * I don't use the java sequencer, I just want the timestamped events.
 */
public class MidiReader {
    private final Track[] m_tracks;

    public MidiReader() {
        // this("src/main/deploy/scales.mid");
        this("scales.mid");
    }

    public MidiReader(String filepath) {
        m_tracks = makeTracks(filepath);
    }

    public Track[] getTracks() {
        return m_tracks;
    }

    /** Dumps all the tracks to stdout. */
    public void print() {
        for (Track track : m_tracks) {
            for (int iEvent = 0; iEvent < track.size(); ++iEvent) {
                MidiEvent event = track.get(iEvent);
                MidiMessage msg = event.getMessage();
                long tick = event.getTick();
                if (!(msg instanceof ShortMessage)) {
                    System.out.println("Other message: " + msg.getClass());
                    continue;
                }
                ShortMessage sm = (ShortMessage) msg;

                if (sm.getCommand() != ShortMessage.NOTE_ON) {
                    System.out.println("Command:" + sm.getCommand());
                    continue;
                }

                int channel = sm.getChannel();
                int key = sm.getData1();
                int velocity = sm.getData2();
                if (velocity > 0) {
                    System.out.println(
                            "Ch " + channel + " tick " + tick + " Attack  " + " key " + key + " v " + velocity);
                } else {
                    System.out.println(
                            "Ch " + channel + " tick " + tick + " Release " + " key " + key);
                }
            }
        }
    }

    /** Returns an array of tracks, empty if something goes wrong. */
    private Track[] makeTracks(String filepath) {
        return MidiUtil.sequence(filepath).getTracks();
    }
}
