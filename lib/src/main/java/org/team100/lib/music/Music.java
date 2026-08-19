package org.team100.lib.music;

import java.util.List;

import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;

/** This is a subsystem so that we can require it */
public interface Music extends Subsystem {
    /** Unison */
    Command play(double freq);

    List<Player> players();
}
