package org.team100.lib.music;

/**
 * Each player plays a different note.
 * 
 * TODO: Maybe just use the array.
 */
public class Polyphony {
    private static final boolean DEBUG = false;
    private final Player m_players[];

    public Polyphony(Player... players) {
        m_players = players;
    }

    /** Player i plays freq */
    public void play(int i, double freq) {
        if (DEBUG)
            System.out.printf("%d %f\n", i, freq);
        if (i < m_players.length)
            m_players[i].play(freq);
    }
}
