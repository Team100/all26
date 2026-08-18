package org.team100.lib.music;

import java.util.List;

import org.team100.lib.hid.Minilab3Keys;

import edu.wpi.first.wpilibj2.command.Command;

/** Maps one key to one player. */
public class Polyphony extends Command {

    private final Minilab3Keys m_keys;
    private final List<Player> m_players;

    public Polyphony(Minilab3Keys keys, Music music) {
        m_keys = keys;
        m_players = music.players();
        addRequirements(music);
    }

    @Override
    public void execute() {
        List<Note> notes = m_keys.notes();
        for (int i = 0; i < notes.size() && i < m_players.size(); ++i) {
            m_players.get(i).play(notes.get(i).hz);
        }
    }

}
