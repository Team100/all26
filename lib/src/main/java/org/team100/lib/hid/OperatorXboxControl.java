package org.team100.lib.hid;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * This class exists to group inputs for velocity, and to provide nicer method
 * names.
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class OperatorXboxControl {
    private final XboxController m_controller;

    public OperatorXboxControl(int port) {
        m_controller = new XboxController(port);
    }

    public Velocity velocity() {
        return ControlUtil.velocity(
                m_controller::getRightY,
                m_controller::getRightX,
                m_controller::getLeftX,
                0.1,
                0.65);
    }

    public double leftY() {
        return m_controller.getLeftY();
    }

    public boolean leftBumper() {
        return m_controller.getLeftBumperButton();
    }

    public boolean rightBumper() {
        return m_controller.getRightBumperButton();
    }

    /** Right trigger is all the way in */
    public boolean rightTrigger() {
        return m_controller.getRightTriggerAxis() > 0.9;
    }
    public boolean leftTrigger() {
        return m_controller.getLeftTriggerAxis() > 0.9;
    }
}
