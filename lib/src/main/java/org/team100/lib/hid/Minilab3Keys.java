package org.team100.lib.hid;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.music.Note;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Direct mapping of the MiniLab 3 "keys" MIDI interface, which uses both white
 * and black keys.
 * 
 * See https://github.com/Team100/all25/tree/main/console/arduino/minilab3_keys
 */
public class Minilab3Keys {
    private static final Note[] NOTES = {
            Note.C3,
            Note.Cs3,
            Note.D3,
            Note.Ds3,
            Note.E3,
            Note.F3,
            Note.Fs3,
            Note.G3,
            Note.Gs3,
            Note.A3,
            Note.As3,
            Note.B3,
            Note.C4,
            Note.Cs4,
            Note.D4,
            Note.Ds4,
            Note.E4,
            Note.F4,
            Note.Fs4,
            Note.G4,
            Note.Gs4,
            Note.A4,
            Note.As4,
            Note.B4,
            Note.C5
    };
    private final GenericHID m_controller;

    public Minilab3Keys(int port) {
        m_controller = new GenericHID(port);
    }

    public List<Note> notes() {
        int buttons = DriverStation.getStickButtons(m_controller.getPort());
        List<Note> notes = new ArrayList<>();
        for (int i = 0; i < 25; ++i) {
            int x = buttons & (1 << i);
            notes.add(NOTES[x]);
        }
        return notes;
    }

    public boolean c3() {
        return m_controller.getRawButton(1);
    }

    public boolean cs3() {
        return m_controller.getRawButton(2);
    }

    public boolean d3() {
        return m_controller.getRawButton(3);
    }

    public boolean ds3() {
        return m_controller.getRawButton(4);
    }

    public boolean e3() {
        return m_controller.getRawButton(5);
    }

    public boolean f3() {
        return m_controller.getRawButton(6);
    }

    public boolean fs3() {
        return m_controller.getRawButton(7);
    }

    public boolean g3() {
        return m_controller.getRawButton(8);
    }

    public boolean gs3() {
        return m_controller.getRawButton(9);
    }

    public boolean a3() {
        return m_controller.getRawButton(10);
    }

    public boolean as3() {
        return m_controller.getRawButton(11);
    }

    public boolean b3() {
        return m_controller.getRawButton(12);
    }

    public boolean c4() {
        return m_controller.getRawButton(13);
    }

    public boolean cs4() {
        return m_controller.getRawButton(14);
    }

    public boolean d4() {
        return m_controller.getRawButton(15);
    }

    public boolean ds4() {
        return m_controller.getRawButton(16);
    }

    public boolean e4() {
        return m_controller.getRawButton(17);
    }

    public boolean f4() {
        return m_controller.getRawButton(18);
    }

    public boolean fs4() {
        return m_controller.getRawButton(19);
    }

    public boolean g4() {
        return m_controller.getRawButton(20);
    }

    public boolean gs4() {
        return m_controller.getRawButton(21);
    }

    public boolean a4() {
        return m_controller.getRawButton(22);
    }

    public boolean as4() {
        return m_controller.getRawButton(23);
    }

    public boolean b4() {
        return m_controller.getRawButton(24);
    }

    public boolean c5() {
        return m_controller.getRawButton(25);
    }

    public boolean pad1() {
        return m_controller.getRawButton(26);
    }

    public boolean pad2() {
        return m_controller.getRawButton(27);
    }

    public boolean pad3() {
        return m_controller.getRawButton(28);
    }

    public boolean pad4() {
        return m_controller.getRawButton(29);
    }

    public boolean pad5() {
        return m_controller.getRawButton(30);
    }

    public boolean pad6() {
        return m_controller.getRawButton(31);
    }

    public boolean pad7() {
        return m_controller.getRawButton(32);
    }

}
