package org.team100.lib.hid;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ControlUtil {
    /**
     * Mix in the cube of the input; this feature is generally called "expo"
     * in the RC community even though it's not an exponential function.
     * 
     * @param input    range [-1,1]
     * @param fraction how much cubic to add, [0,1]
     */
    public static double expo(double input, double fraction) {
        return (1 - fraction) * input + fraction * input * input * input;
    }

    public static double deadband(double input, double threshold, double maxMagnitude) {
        return MathUtil.applyDeadband(input, threshold, maxMagnitude);
    }

    public static double clamp(double input, double clamp) {
        return MathUtil.clamp(input, -clamp, clamp);
    }

    public static double clamp(double input, double low, double high) {
        return MathUtil.clamp(input, low, high);
    }

    public static double scale(double raw, double negScale, double center, double posScale) {
        double zeroed = raw - center;
        if (zeroed < 0)
            return zeroed / negScale;
        return zeroed / posScale;
    }

    /**
     * Applies expo to the magnitude of the cartesian input. Appropriate for
     * joysticks with round limits, like Xbox controllers.
     * 
     * TODO: add acceleration computation
     */
    public static Velocity velocity(
            DoubleSupplier rightYSupplier,
            DoubleSupplier rightXSupplier,
            DoubleSupplier leftXSupplier,
            double deadband,
            double expo) {
        final double rightY = rightYSupplier.getAsDouble();
        final double rightX = rightXSupplier.getAsDouble();
        final double leftX = leftXSupplier.getAsDouble();
        double dx = 0;
        double dy = 0;
        double x = -1.0 * clamp(rightY, 1);
        double y = -1.0 * clamp(rightX, 1);
        double r = Math.hypot(x, y);
        if (r > deadband) {
            double expoR = expo(r, expo);
            double ratio = expoR / r;
            dx = ratio * x;
            dy = ratio * y;
        } else {
            dx = 0;
            dy = 0;
        }
        double dtheta = expo(deadband(-1.0 * clamp(leftX, 1), deadband, 1), expo);
        return new Velocity(dx, dy, dtheta);
    }

    /**
     * Absolute rotational input. This could be mapped to a turntable or a POV hat
     * or similar. Return null if the control should be ignored.
     * 
     * @param pov angle of POV in degrees or -1 if not pressed.
     */
    public static Rotation2d pov(IntSupplier pov) {
        double desiredAngleDegrees = pov.getAsInt();
        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    private ControlUtil() {
    }
}
