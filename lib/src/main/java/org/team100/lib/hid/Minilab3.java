package org.team100.lib.hid;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Direct mapping of the MiniLab 3 MIDI interface.
 * 
 * See https://github.com/Team100/all25/tree/main/console/arduino/minilab3
 */
public class Minilab3 {
    private final GenericHID m_controller;

    public Minilab3(int port) {
        m_controller = new GenericHID(port);
    }

    public boolean c3() {
        return m_controller.getRawButton(1);
    }

    public boolean d3() {
        return m_controller.getRawButton(2);
    }

    public boolean e3() {
        return m_controller.getRawButton(3);
    }

    public boolean f3() {
        return m_controller.getRawButton(4);
    }

    public boolean g3() {
        return m_controller.getRawButton(5);
    }

    public boolean a3() {
        return m_controller.getRawButton(6);
    }

    public boolean b3() {
        return m_controller.getRawButton(7);
    }

    public boolean c4() {
        return m_controller.getRawButton(8);
    }

    public boolean d4() {
        return m_controller.getRawButton(9);
    }

    public boolean e4() {
        return m_controller.getRawButton(10);
    }

    public boolean f4() {
        return m_controller.getRawButton(11);
    }

    public boolean g4() {
        return m_controller.getRawButton(12);
    }

    public boolean a4() {
        return m_controller.getRawButton(13);
    }

    public boolean b4() {
        return m_controller.getRawButton(14);
    }

    public boolean c5() {
        return m_controller.getRawButton(15);
    }

}
