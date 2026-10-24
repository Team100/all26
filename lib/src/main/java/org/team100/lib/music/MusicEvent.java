package org.team100.lib.music;

/** Target of midi translator.  Zero means rest. */
public record MusicEvent(double t, double freq) {

}
