package org.team100.lib.music;

import java.util.Arrays;
import java.util.List;

/** All players play the same note. */
public class Unison implements Player {
    private final Player m_players[];

    public Unison(Player... players) {
        m_players = players;
    }

    @Override
    public void play(double freq) {
        Arrays.stream(m_players).forEach(p -> play(freq));
    }

    @Override
    public List<Player> players() {
        return Arrays.stream(m_players).flatMap(p -> p.players().stream()).toList();
    }

}
