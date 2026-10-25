package org.team100.lib.music;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.Filesystem;

public class MidiUtil {
    public static Sequence sequence(String filepath) {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(filepath);
        System.out.printf("Trying to load path %s\n", path.toAbsolutePath().toString());
        try (InputStream is = Files.newInputStream(path)) {
            return MidiSystem.getSequence(is);
        } catch (IOException | InvalidMidiDataException e) {
            throw new RuntimeException(e);
        }
    }

    public static double ticksToSec(long ticks, double beatsPerMinute, double ticksPerBeat) {
        if (beatsPerMinute < 1)
            return 0;
        double ticksPerMinute = beatsPerMinute * ticksPerBeat;
        double ticksPerSec = ticksPerMinute / 60.0;
        return ticks / ticksPerSec;
    }
}
