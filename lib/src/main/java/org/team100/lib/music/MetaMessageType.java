package org.team100.lib.music;

import java.util.Map;
import java.util.TreeMap;

public enum MetaMessageType {
    SEQUENCE_NUMBER(0x00),
    TEXT(0x01),
    COPYRIGHT_NOTICE(0x02),
    TRACK_NAME(0x03),
    INSTRUMENT_NAME(0x04),
    LYRIC(0x05),
    MARKER(0x06),
    CUE_POINT(0x07),
    MIDI_CHANNEL_PREFIX(0x20),
    MIDI_PORT(0x21),
    END_OF_TRACK(0x2F),
    SET_TEMPO(0x51),
    SMPTE_OFFSET(0x54),
    TIME_SIGNATURE(0x58),
    KEY_SIGNATURE(0x59),
    SEQUENCER_SPECIFIC(0x7F),
    UNKNOWN(256);

    private static Map<Integer, MetaMessageType> all = new TreeMap<>();
    static {
        for (MetaMessageType mm : values()) {
            all.put(mm.value, mm);
        }
    }

    private final int value;

    MetaMessageType(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

    public static MetaMessageType get(int type) {
        MetaMessageType m = all.get(type);
        if (m == null)
            return MetaMessageType.UNKNOWN;
        return m;
    }
}
