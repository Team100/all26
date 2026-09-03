package org.team100.lib.config;

/**
 * Friction model for static, dynamic, and viscous friction.
 * 
 * Applicable for motor feedforward.
 * 
 * These values describe the entire mechanism, not just the motor, which is why
 * it's not a property of the motor class.
 * 
 * @see https://mogi.bme.hu/TAMOP/robot_applications/ch07.html
 * @see https://en.wikipedia.org/wiki/Friction
 * @see https://en.wikipedia.org/wiki/Stribeck_curve
 * @see https://engee.com/helpcenter/stable/en/fmod-mechanical-translational-elements/translational-friction.html
 */
public class Friction {
    /** Volts */
    private final double kS;
    /** Volts */
    private final double kD;
    /** Volt-sec/rad */
    private final double kV;
    /** rad/sec */
    private final double vS;

    /**
     * @param kS Static friction. Voltage to just barely get the mechanism moving
     *           from a stop. Measure this using any voltage control, literally
     *           iterate to find the "just get started" voltage.
     * @param kD Dynamic friction. Voltage to just barely keep the mechanism
     *           moving, independent of speed. Measure the voltage intercept of the
     *           V-vs-omega curve.
     * @param kV Viscous friction. Constant to compute voltage to keep moving at a
     *           constant velocity. Units are Volt-sec/rad. Measure this by plotting
     *           the V-omega curve with the whole mechanism attached -- you'll find
     *           a different slope than with the motor alone. The difference in
     *           slope is kV.
     * @param vS Velocity threshold for static friction, rad/s. This is very hard to
     *           measure, just use a small number, like 0.5.
     */
    public Friction(
            double kS,
            double kD,
            double kV,
            double vS) {
        if (kS < kD)
            throw new IllegalArgumentException("static friction is always at least as high as dynamic friction");
        this.kS = kS;
        this.kD = kD;
        this.kV = kV;
        this.vS = vS;
    }

    /**
     * Voltage to balance friction (i.e. this has the same sign as the supplied
     * speed).
     * 
     * Includes viscous friction (proportional to speed), dynamic friction (constant
     * while moving), and static friction (constant while almost stopped).
     * 
     * @param motorRad_S setpoint speed rad/s
     */
    public double frictionFFVolts(double motorRad_S) {
        double viscous = kV * motorRad_S;
        double direction = Math.signum(motorRad_S);
        if (Math.abs(motorRad_S) < vS) {
            return viscous + kS * direction;
        }
        return viscous + kD * direction;
    }
}
