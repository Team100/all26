package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * This class exists to group inputs for velocity, and to provide nicer method
 * names.
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl {
    private final XboxController m_controller;

    public DriverXboxControl(int port) {
        m_controller = new XboxController(port);
    }

    /**
     * "RC mode 2" control:
     * * right Y (axis 5) is the field "X" direction, ahead
     * * right X (axis 4) is the field "Y" direction, to the left
     * * left X (axis 0) is rotation, counterclockwise
     * 
     * TODO: add acceleration computation.
     */
    public Velocity velocity() {
        return ControlUtil.velocity(
                m_controller::getRightY,
                m_controller::getRightX,
                m_controller::getLeftX,
                0.1,
                0.65);
    }

    /** Axis 5 */
    public double rightY() {
        return m_controller.getRightY();
    }

    /** Axis 4 */
    public double rightX() {
        return m_controller.getRightX();
    }

    /** Axis 0 */
    public double leftX() {
        return m_controller.getLeftX();
    }

    public Rotation2d pov() {
        return ControlUtil.pov(m_controller::getPOV);
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
        return m_controller.getAButton();
    }

    /** Button 2 */
    public boolean b() {
        return m_controller.getBButton();
    }

    /** Button 3 */
    public boolean x() {
        return m_controller.getXButton();
    }

    /** Button 4 */
    public boolean y() {
        return m_controller.getYButton();
    }

    /** Axis 1 */
    public double leftY() {
        return m_controller.getLeftY();
    }

}
