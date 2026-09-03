package org.team100.lib.hid;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Rotation2d;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * We use "Mode 2" stick arrangement, with cartesian motion on the right
 * and yaw on the left.
 * 
 * Smooths the input a little, to compensate for the low sampling rate.
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl {

    // width of the stick filter in clock cycles
    private static final int BOXCAR = 4;
    private final Gamepad m_controller;
    /**
     * Controls are sampled at half the main clock rate,
     * which makes the "acceleration" computation return zero
     * half the time. So this smooths it out.
     */
    private final LinearFilter m_filterRightY;
    private final LinearFilter m_filterRightX;
    private final LinearFilter m_filterLeftX;

    private final DoubleCache m_rightY;
    private final DoubleCache m_rightX;
    private final DoubleCache m_leftX;

    private final DoubleLogger m_log_rightY;
    private final DoubleLogger m_log_rightX;
    private final DoubleLogger m_log_leftX;

    public DriverXboxControl(LoggerFactory parent, int port) {
        LoggerFactory log = parent.type(this);
        m_controller = new Gamepad(port);
        // Match the control sampling rate.
        m_filterRightY = LinearFilter.movingAverage(BOXCAR);
        m_filterRightX = LinearFilter.movingAverage(BOXCAR);
        m_filterLeftX = LinearFilter.movingAverage(BOXCAR);
        m_rightY = Cache.ofDouble(this::rightY);
        m_rightX = Cache.ofDouble(this::rightX);
        m_leftX = Cache.ofDouble(this::leftX);
        m_log_rightY = log.doubleLogger(Level.DEBUG, "right Y");
        m_log_rightX = log.doubleLogger(Level.DEBUG, "right X");
        m_log_leftX = log.doubleLogger(Level.DEBUG, "left X");
    }

    /**
     * "RC mode 2" control:
     * * right Y (axis 5) is the field "X" direction, ahead
     * * right X (axis 4) is the field "Y" direction, to the left
     * * left X (axis 0) is rotation, counterclockwise
     */
    public Velocity velocity() {
        return ControlUtil.velocity(
                m_rightY,
                m_rightX,
                m_leftX,
                0.1,
                0.65);
    }

    public void periodic() {
        m_log_rightY.log(m_rightY);
        m_log_rightX.log(m_rightX);
        m_log_leftX.log(m_leftX);
    }

    /** Axis 5 */
    public double rightY() {
        return m_filterRightY.calculate(m_controller.getRightY());
    }

    /** Axis 4 */
    public double rightX() {
        return m_filterRightX.calculate(m_controller.getRightX());
    }

    /** Axis 0 */
    public double leftX() {
        return m_filterLeftX.calculate(m_controller.getLeftX());
    }

    public Rotation2d pov() {
        // TODO: fix for 2027
        return Rotation2d.kZero;
        // return ControlUtil.pov(m_controller::getPOV);
    }

    /** POV switch pressed on the top */
    public boolean povUp() {
        Rotation2d pov = pov();
        if (pov == null)
            return false;
        return pov.equals(Rotation2d.kZero);
    }

    /** POV switch pressed on the bottom */
    public boolean povDown() {
        Rotation2d pov = pov();
        if (pov == null)
            return false;
        return pov.equals(Rotation2d.kPi);
    }

    /** Button 7 */
    public boolean back() {
        return m_controller.getBackButton();
    }

    /** Button 8 */
    public boolean start() {
        return m_controller.getStartButton();
    }

    /** Left trigger (axis 2) is all the way in */
    public boolean leftTrigger() {
        return m_controller.getLeftTriggerAxis() > 0.9;
    }

    /** Right trigger (axis 3) is all the way in. */
    public boolean rightTrigger() {
        return m_controller.getRightTriggerAxis() > 0.9;
    }

    /** Button 5 */
    public boolean leftBumper() {
        return m_controller.getLeftBumperButton();
    }

    /** Button 6 */
    public boolean rightBumper() {
        return m_controller.getRightBumperButton();
    }

    /** Button 1 */
    public boolean a() {
        return m_controller.getSouthFaceButton();
    }

    /** Button 2 */
    public boolean b() {
        return m_controller.getEastFaceButton();
    }

    /** Button 3 */
    public boolean x() {
        return m_controller.getWestFaceButton();
    }

    /** Button 4 */
    public boolean y() {
        return m_controller.getNorthFaceButton();
    }

    /** Axis 1 */
    public double leftY() {
        return m_controller.getLeftY();
    }

}
