package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * VKB Joystick.
 * 
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class VKBJoystick {
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 0.5;

    private final GenericHID m_hid;

    protected VKBJoystick() {
        m_hid = new GenericHID(0);
    }

    public boolean useReefLock() {
        return button(0); // trigger halfway down
    }

    public boolean test() {
        return button(3); // red thumb
    }

    public boolean resetRotation0() {
        return button(7); // "F1"
    }

    public boolean resetRotation180() {
        return button(8); // "F2"
    }

    public boolean button5() {
        // return button(5);
        return false;
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */

    public Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(axis(1), 1), DEADBAND, 1), EXPO);
        double dy = expo(deadband(-1.0 * clamp(axis(0), 1), DEADBAND, 1), EXPO);
        double dtheta = expo(deadband(clamp(axis(5), 1), DEADBAND, 1), EXPO);
        return new Velocity(dx, dy, dtheta);
    }

    public Rotation2d desiredRotation() {
        // POV 2 is the center one
        double desiredAngleDegrees = m_hid.getPOV(2);
        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    public boolean toReef() {
        return button(8);
    }

    private double axis(int axis) {
        return m_hid.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_hid.getRawButton(button);
    }
}
