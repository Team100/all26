package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;
import static org.team100.lib.hid.ControlUtil.scale;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.math.filter.LinearFilter;

/**
 * The Spektrum InterLinkDX controller is an RC-style control with a USB
 * interface.
 * 
 * We use "Mode 2" stick arrangement, with cartesian motion on the right
 * and yaw on the left.
 * 
 * Smooths the input a little, to compensate for the low sampling rate.
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
    // width of the stick filter in clock cycles
    private static final int BOXCAR = 4;
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;
    private static final double SLOW = 0.25;

    private final GenericHID m_hid;
    /**
     * Controls are sampled at half the main clock rate,
     * which makes the "acceleration" computation return zero
     * half the time. So this smooths it out.
     */
    private final LinearFilter m_filterRightX;
    private final LinearFilter m_filterRightY;
    private final LinearFilter m_filterLeftX;

    private final DoubleCache m_rightY;
    private final DoubleCache m_rightX;
    private final DoubleCache m_leftX;

    private final DoubleLogger m_log_rightY;
    private final DoubleLogger m_log_rightX;
    private final DoubleLogger m_log_leftX;

    public InterLinkDX(LoggerFactory parent, int port) {
        LoggerFactory log = parent.type(this);
        m_hid = new GenericHID(port);
        // Match the control sampling rate.
        m_filterRightX = LinearFilter.movingAverage(BOXCAR);
        m_filterRightY = LinearFilter.movingAverage(BOXCAR);
        m_filterLeftX = LinearFilter.movingAverage(BOXCAR);
        m_rightY = Cache.ofDouble(this::rightY);
        m_rightX = Cache.ofDouble(this::rightX);
        m_leftX = Cache.ofDouble(this::leftX);
        m_log_rightY = log.doubleLogger(Level.DEBUG, "right Y");
        m_log_rightX = log.doubleLogger(Level.DEBUG, "right X");
        m_log_leftX = log.doubleLogger(Level.DEBUG, "left X");
    }

    public Velocity velocity() {
        double dx = expo(deadband(
                clamp(scale(m_rightY.getAsDouble(), 0.836, 0.031, 0.900), 1),
                DEADBAND, 1),
                EXPO);
        double dy = expo(deadband(
                -1.0 * clamp(scale(m_rightX.getAsDouble(), 0.859, -0.008, 0.827), 1),
                DEADBAND, 1),
                EXPO);
        double dtheta = expo(deadband(
                -1.0 * clamp(scale(m_leftX.getAsDouble(), 0.812, 0.0, 0.850), 1),
                DEADBAND, 1),
                EXPO);
        if (button(1))
            return new Velocity(SLOW * dx, SLOW * dy, SLOW * dtheta);
        return new Velocity(dx, dy, dtheta);
    }

    public void periodic() {
        m_log_rightY.log(m_rightY);
        m_log_rightX.log(m_rightX);
        m_log_leftX.log(m_leftX);
    }

    private double leftX() {
        return m_filterLeftX.calculate(axis(0));
    }

    private double rightX() {
        return m_filterRightX.calculate(axis(3));
    }

    private double rightY() {
        return m_filterRightY.calculate(axis(4));
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

    ///////////////////////////////////////////////////////////

    /** Get raw axis value. */
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