package org.team100.lib.framework;

import org.wpilib.hardware.discrete.PWM;
import org.wpilib.util.sendable.SendableBuilder;

/** We use Servo.java. */
public class Servo extends PWM {

    // private static final double kMaxServoAngle = 180.0;
    // private static final double kMinServoAngle = 0.0;

    // private static final int kDefaultMaxServoPWM = 2400;
    // private static final int kDefaultMinServoPWM = 600;

    /**
     * Constructor.
     *
     * <p>
     * By default, {@value #kDefaultMaxServoPWM} ms is used as the max PWM value and
     * {@value
     * #kDefaultMinServoPWM} ms is used as the minPWM value.
     *
     * @param channel The PWM channel to which the servo is attached. 0-9 are
     *                on-board, 10-19 are on
     *                the MXP port
     */
    @SuppressWarnings("this-escape")
    public Servo(final int channel) {
        super(channel);
        // TODO: implement this
    }

    public void set(double value) {
        // TODO: implement this
    }

    public double get() {
        // TODO: implement this
        // return getPosition();
        return 0;
    }

    public void setAngle(double degrees) {
        // if (degrees < kMinServoAngle) {
        // degrees = kMinServoAngle;
        // } else if (degrees > kMaxServoAngle) {
        // degrees = kMaxServoAngle;
        // }

        // setPosition((degrees - kMinServoAngle) / getServoAngleRange());
    }

    public double getAngle() {
        // TODO: implement this
        // return getPosition() * getServoAngleRange() + kMinServoAngle;
        return 0;
    }

    @SuppressWarnings("unused")
    private double getServoAngleRange() {
        // TODO: implement this
        // return kMaxServoAngle - kMinServoAngle;
        return 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Servo");
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}