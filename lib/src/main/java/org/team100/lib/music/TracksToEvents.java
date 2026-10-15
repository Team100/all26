package org.team100.lib.music;

import java.util.ArrayList;
import java.util.List;

import javax.sound.midi.MidiEvent;
import javax.sound.midi.MidiMessage;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Track;

/** Translate MIDI tracks to events. */
public class TracksToEvents {

    public TracksToEvents() {

    }

    public List<MusicEvent> toEvents(Track[] tracks) {
        List<MusicEvent> events = new ArrayList<>();
        for (Track track : tracks) {
            for (int iEvent = 0; iEvent < track.size(); ++iEvent) {
                MidiEvent event = track.get(iEvent);
                MidiMessage msg = event.getMessage();

                if (!(msg instanceof ShortMessage))
                    continue;

                ShortMessage sm = (ShortMessage) msg;
                if (sm.getCommand() != ShortMessage.NOTE_ON)
                    continue;

                int channel = sm.getChannel();
                // for now, just use channel zero.
                if (channel != 0)
                    continue;

                // a "tick" is a millisecond, i think.
                long tick = event.getTick();
                int key = sm.getData1();

                Note note = Note.get(key);

                double tSec = tick / 1000;

                int velocity = sm.getData2();
                if (velocity > 0) {
                    // play the note
                    events.add(new MusicEvent(tSec, note.hz));
                } else {
                    events.add(new MusicEvent(tSec, 0));
                }
            }
        }
        return events;
    }
}
