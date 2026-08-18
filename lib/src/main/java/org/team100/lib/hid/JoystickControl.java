package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Experiment for driving swerve with the big joystick.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public abstract class JoystickControl {
    private static final boolean DEBUG = false;
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;

    private final GenericHID m_controller;

    protected JoystickControl() {
        m_controller = new GenericHID(0);
    }

    public boolean test() {
        return false;
        // return button(3);
    }

    public boolean resetRotation0() {
        // return button(2);
        return false;
    }

    public boolean resetRotation180() {
        return false;
        // return button(3);
    }

    public boolean button4() {
        return button(4);
    }

    public boolean button5() {
        return button(5);
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    public Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(1), 1), DEADBAND, 1), EXPO);
        double dy = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(0), 1), DEADBAND, 1), EXPO);
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(2), 1), DEADBAND, 1), EXPO);
        Velocity velocity = new Velocity(dx, dy, dtheta);
        if (DEBUG) {
            System.out.printf("JoystickControl %s\n", velocity);
        }
        return velocity;
    }

    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();
        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    private boolean button(int button) {
        return m_controller.getRawButton(button);
    }
}
