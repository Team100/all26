package org.team100.lib.music;

import java.util.Map;
import java.util.TreeMap;

/** see https://www.songstuff.com/recording/article/midi-message-format */
public enum ShortMessageCommand {

    NOTE_OFF(0x80), // [note number, velocity]
    NOTE_ON(0x90), // [note number, velocity]
    POLY_PRESSURE(0xA0), // [note number, pressure]
    CONTROL_CHANGE(0xB0), // [controller number, data]
    PROGRAM_CHANGE(0xC0), // [program number, (unused)]
    CHANNEL_PRESSURE(0xD0), // [pressure, (unused)]
    PITCH_BEND(0xE0), // [lsb, msb]
    UNKNOWN(256);

    private static Map<Integer, ShortMessageCommand> all = new TreeMap<>();
    static {
        for (ShortMessageCommand mm : values()) {
            all.put(mm.value, mm);
        }
    }

    private final int value;

    ShortMessageCommand(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

    public static ShortMessageCommand get(int type) {
        ShortMessageCommand m = all.get(type);
        if (m == null)
            return ShortMessageCommand.UNKNOWN;
        return m;
    }
}
