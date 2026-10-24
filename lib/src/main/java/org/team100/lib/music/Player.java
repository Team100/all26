package org.team100.lib.music;

import java.util.List;

/** A single thing that can play a note, i.e. a motor. */
public interface Player {
    /** Unison */
    void play(double freq);

    default List<Player> players() {
        return List.of(this);
    }
}