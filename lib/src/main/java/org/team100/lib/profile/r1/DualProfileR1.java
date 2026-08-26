package org.team100.lib.profile.r1;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

/**
 * Uses a trapezoid profile for low (current-limited) speed.
 * Uses an exponential profile for high (back-EMF-limited) speed.
 * 
 * The trapezoid profile maximum acceleration relates to the current-limited
 * torque.
 * 
 * The exponential profile maximum acceleration relates to the *unlimited* stall
 * torque.
 */
public class DualProfileR1 implements ProfileR1 {
    private static final boolean DEBUG = false;
    private final double m_maxVel;
    /** Unlimited stall torque */
    private final double m_stallAccel;
    /** Current-limited torque */
    private final double m_limitedAccel;
    private final WPITrapezoidProfileR1 m_trapezoid;
    private final WPIExponentialProfileR1 m_exponential;
    /** Speed where the torque curves cross */
    private final double m_limit;

    /** Typically the stall accel is double the limited accel. */
    public DualProfileR1(
            double maxVel,
            double limitedAccel,
            double stallAccel) {
        m_maxVel = maxVel;
        m_stallAccel = stallAccel;
        m_limitedAccel = limitedAccel;
        m_trapezoid = new WPITrapezoidProfileR1(maxVel, limitedAccel);
        m_exponential = new WPIExponentialProfileR1(maxVel, stallAccel);
        m_limit = (1 - m_limitedAccel / m_stallAccel) * maxVel;
    }

    static boolean isAccel(ControlR1 initial, ControlR1 setpoint) {
        double initialV = initial.v();
        double setpointV = setpoint.v();
        boolean isAccel = Math.abs(setpointV) > Math.abs(initialV)
                && Math.signum(setpointV) == Math.signum(initialV);
        if (DEBUG) {
            System.out.printf("initial %5.2f setpoint %5.2f isAccel %b\n", initialV, setpointV, isAccel);
        }
        return isAccel;
    }

    @Override
    public ControlR1 calculate(double dt, ControlR1 initial, StateR1 goal) {
        ControlR1 trapezoid = m_trapezoid.calculate(dt, initial, goal);
        ControlR1 exponential = m_exponential.calculate(dt, initial, goal);
        if (!isAccel(initial, exponential)) {
            // exponential decel is more accurate ("plugging" torque is higher than stall)
            return exponential;
        }
        if (initial.v() < m_limit) {
            // Low speed is current limited.
            return trapezoid;
        }
        // high speed is back-EMF limited.
        return exponential;
    }

    @Override
    public ProfileR1 scale(double s) {
        return new DualProfileR1(
                m_maxVel, s * m_limitedAccel, s * m_stallAccel);
    }

    public double getMaxVelocity() {
        return m_maxVel;
    }
}
