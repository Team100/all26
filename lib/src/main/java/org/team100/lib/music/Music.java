package org.team100.lib.music;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** This is a subsystem so that we can require it */
public interface Music extends Subsystem {
    /** Unison */
    Command play(double freq);

    List<Player> players();
}
