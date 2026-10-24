package org.team100.lib.music;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** A subsystem that can play a note. */
public interface Music extends Subsystem {
    /** Play sound in unison. */
    Command play(double freq);

    List<Player> players();
}
