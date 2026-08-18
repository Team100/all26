package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;
import static org.team100.lib.hid.ControlUtil.scale;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * The Spektrum InterLinkDX controller is an RC-style control with a USB
 * interface.
 * 
 * HARDWARE
 * 
 * The control layout is as follows:
 * 
 * left x: axis 0
 * left y: axis 1
 * left rear: axis 2
 * right x: axis 3
 * right y: axis 4
 * right rear: axis 5
 * R knob: axis 7
 * 
 * 
 * button 1: switch "A" 1 (0 is off)
 * button 2: switch "B" 0
 * button 3: switch "B" 2 (1 is off)
 * button 4: switch "C" 0
 * button 5: switch "C" 2 (1 is off)
 * button 6: switch "D" 0
 * button 7: switch "D" 2 (1 is off)
 * button 8: switch "F" 2
 * button 9: switch "F" 0 (1 is off)
 * button 10: switch "G" 2
 * button 11: switch "G" 0 (1 is off)
 * button 12: switch "H" 1 (0 is off)
 * button 13: switch "I"
 * button 14: switch "RESET"
 * button 15: switch "CANCEL"
 * button 16: switch "SELECT" push
 * button 17: switch "SELECT" to the left
 * button 18: switch "SELECT" to the right
 * button 19: left bottom trim left
 * button 20: left bottom trim right
 * button 21: left side trim down
 * button 22: left side trim up
 * button 23: right bottom trim left
 * button 24: right bottom trim right
 * button 25: right side trim down
 * button 26: right side trim up
 * button 27: ??
 * 
 * @see https://my.spektrumrc.com/ProdInfo/Files/SPMRFTX1-Manual-EN.pdf
 */

public class InterLinkDX {
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;
    private static final double SLOW = 0.25;

    private final GenericHID m_hid;

    public InterLinkDX(int port) {
        m_hid = new GenericHID(port);
    }

    public Velocity velocity() {
        double dx = expo(deadband(
                clamp(scale(axis(4), 0.836, 0.031, 0.900), 1),
                DEADBAND, 1),
                EXPO);
        double dy = expo(deadband(
                -1.0 * clamp(scale(axis(3), 0.859, -0.008, 0.827), 1),
                DEADBAND, 1),
                EXPO);
        double dtheta = expo(deadband(
                -1.0 * clamp(scale(axis(0), 0.812, 0.0, 0.850), 1),
                DEADBAND, 1),
                EXPO);
        if (button(1))
            return new Velocity(SLOW * dx, SLOW * dy, SLOW * dtheta);
        return new Velocity(dx, dy, dtheta);
    }

    /** "A" is on the left side on the back corner. 1 is up/in. */
    public boolean a1() {
        return button(1);
    }

    /** "C" is on the left side on the front face. 0 is up/forward. */
    public boolean c0() {
        return button(4);
    }

    /** "C" is on the left side on the front face. 2 is down/back. */
    public boolean c2() {
        return button(5);
    }

    /** "I" is the button on the upper left, on the top face. */
    public boolean i() {
        return button(13);
    }

    public boolean reset() {
        return button(14);
    }

    public boolean cancel() {
        return button(15);
    }

    ////////////////////////////////////////////////////////////

    private double axis(int axis) {
        return m_hid.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_hid.getRawButton(button);
    }

    public boolean back() {
        return cancel();
    }

    public boolean start() {
        return reset();
    }

    public boolean povDown() {
        return false;
    }

    public boolean rightTrigger() {
        return false;
    }

    public boolean x() {
        return false;
    }

    public boolean a() {
        return false;
    }

    public boolean b() {
        return false;
    }

    public boolean y() {
        return false;
    }

    public boolean leftBumper() {
        return false;
    }
}