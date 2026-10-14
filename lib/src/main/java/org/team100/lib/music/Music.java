package org.team100.lib.music;

import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;

/** A subsystem that can play a note. */
public interface Music extends Subsystem {
    /** Play sound in unison. */
    Command play(double freq);

    List<Player> players();
}
