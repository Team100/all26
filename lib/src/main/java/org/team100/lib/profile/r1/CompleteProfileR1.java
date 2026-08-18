package org.team100.lib.profile.r1;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * A simple profile with all the things we want from a motion profile for
 * mechanisms. There are six parameters:
 * 
 * * maximum acceleration
 * * maximum deceleration (typically higher than accel, "plugging")
 * * maximum velocity
 * * "stall" acceleration for calculating back EMF
 * * maximum jerk for takeoff (optional)
 * * maximum jerk for landing (optional)
 * 
 * This works by precalculating the "goal" path on instantiation, and
 * calculating the initial path dynamically. When the initial state is close to
 * the goal path, then states are interpolated from it.
 * 
 * The initial state can be anything; the goal is stationary so it can be
 * precalculated, and this is our only real use-case anyway.
 * 
 * It's analogous to a sliding-mode controller: maximum effort to reach a
 * curve, and then moving alnog it.
 * 
 * See https://www.desmos.com/calculator/jnc7u3jg11 for useful curves.
 * 
 * See
 * https://docs.google.com/spreadsheets/d/1JdKViVSTEMZ0dRS8broub4P-f0eA6STRHHzoV0U4N5M/edit?gid=2097479642#gid=2097479642
 * for output of this model
 */
public class CompleteProfileR1 implements ProfileR1 {
    private static final boolean DEBUG = false;

    /** Time step for initializing the (fixed) goal path */
    private static final double DT = 0.01;
    // Extends maxV far away.
    private static final double FAR_AWAY = 1000;

    private final LoggerFactory m_log;
    private final double m_maxV;
    private final double m_maxAUnscaled;
    private final double m_maxDUnscaled;
    private final double m_stallAUnscaled;
    private final double m_takeoffJ;
    private final double m_landingJ;
    private final double m_scale;
    private final double m_tolerance;
    final InterpolatingTreeMap<Double, ControlR1> m_byDistance;

    /**
     * Too-low a tolerance will produce chatter. Too-high a tolerance will produce a
     * little hiccup at the goal.
     * 
     * Jerk in the middle of the path is unlimited, because it seems complicated to
     * add a limit there.
     * 
     * @param maxV      max velocity
     * @param maxA      max acceleration (current-limited, for initial path)
     * @param maxD      max decel (higher than accel, only used for goal path)
     * @param maxStallA theoretical stall acceleration, for calculating back-EMF
     * @param takeoffJ  max jerk for takeoff, zero for unlimited
     * @param landingJ  max jerk for landing, zero for unlimited
     * @param tolerance this close to the switching curve to be on it.
     *                  also used to sense "at goal"
     */
    public CompleteProfileR1(
            LoggerFactory log, double maxV, double maxA, double maxD, double stallA,
            double takeoffJ, double landingJ, double tolerance) {
        if (maxV <= 0)
            throw new IllegalArgumentException("max V must be positive");
        if (maxA <= 0)
            throw new IllegalArgumentException("max A must be positive");
        if (maxD <= 0)
            throw new IllegalArgumentException("max D must be positive");
        if (stallA <= 0)
            throw new IllegalArgumentException("stall A must be positive");
        if (takeoffJ < 0)
            throw new IllegalArgumentException("takeoff J may not be negative");
        if (landingJ < 0)
            throw new IllegalArgumentException("landing J may not be positive");
        m_log = log;
        m_maxV =  maxV;
        m_maxAUnscaled =  maxA;
        m_maxDUnscaled =  maxD;
        m_stallAUnscaled =  stallA;
        m_takeoffJ =  takeoffJ;
        m_landingJ =  landingJ;
        m_scale = 1.0;
        m_tolerance =  tolerance;
        m_byDistance = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ControlR1::interpolate);
        init();
    }

    public CompleteProfileR1(
            LoggerFactory log, double maxV, double maxA, double maxD, double stallA,
            double takeoffJ, double landingJ, double scale, double tolerance) {
        m_log = log;
        m_maxV = maxV;
        m_maxAUnscaled = maxA;
        m_maxDUnscaled = maxD;
        m_stallAUnscaled = stallA;
        m_takeoffJ = takeoffJ;
        m_landingJ = landingJ;
        m_scale = scale;
        m_tolerance = tolerance;
        m_byDistance = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ControlR1::interpolate);
        init();
    }

    void update(double ignored) {
        init();
    }

    /**
     * Compute the goal path. This is done on instantiation.
     */
    void init() {
        m_byDistance.clear();
        // This is the goal state, zero control here.
        ControlR1 control = new ControlR1();
        put(0.0, control);
        // Far-away points so that the interpolator always yields maxV.
        put(0.0, new ControlR1(-FAR_AWAY, m_maxV, 0));
        put(0.0, new ControlR1(FAR_AWAY, -m_maxV, 0));
        // control from the left, so deceleration, walking back in time
        // t is just for debugging
        double t = 0;
        for (int i = 1; i < 1000; ++i) {
            if (MathUtil.isNear(control.v(), m_maxV, m_tolerance)) {
                // we're already cruising. keep cruising.
                control = new ControlR1(
                        control.x() - m_maxV * DT,
                        m_maxV,
                        0);
                t += DT;
                put(t, control);
            } else {
                double jerkLimitedA = jerkLimitedAccel(control);
                double nextV = control.v() - jerkLimitedA * DT;
                if (nextV > m_maxV) {
                    // maxV is achieved within DT
                    // how long does it take to get there?
                    double dt = -1.0 * (m_maxV - control.v()) / jerkLimitedA;
                    t += dt;
                    // this should be exactly at the corner.
                    control = new ControlR1(
                            control.x() - control.v() * dt + 0.5 * jerkLimitedA * dt * dt,
                            m_maxV,
                            jerkLimitedA);
                    put(t, control);
                    // this is zero accel, epsilon away, so that the interpolator doesn't try to
                    // match the full-accel at the corner.
                    ControlR1 corner = new ControlR1(
                            control.x() - 1e-3,
                            m_maxV,
                            0);
                    put(t, corner);
                    // the "far away" points should take care of the rest.
                    break;
                } else {
                    // Haven't reached maxV yet, keep going on the decel path.
                    control = new ControlR1(
                            control.x() - control.v() * DT + 0.5 * jerkLimitedA * DT * DT,
                            nextV,
                            jerkLimitedA);
                    t += DT;
                    put(t, control);
                }
            }
        }
    }

    @Override
    public ControlR1 calculate(double dt, ControlR1 setpoint, ModelR1 goal) {
        if (Math.abs(goal.v()) > 1e-6)
            throw new IllegalArgumentException("This profile works only with stationary goals.");

        // Negative togo => setpoint is to the left.
        final double togo = setpoint.x() - goal.x();
        if (MathUtil.isNear(0, togo, m_tolerance)) {
            // Within tolerance of goal
            return goal.control();
        }

        final double maxA = accel(dt, setpoint);
        final ControlR1 lerp = m_byDistance.get(togo);

        // When imagining how this works, it's good to have the phase space diagram in
        // front of you. The "move right" and "move left" cases are duplicated here,
        // rather than the usual scheme of inverting the profile, in the interest of
        // clarity.

        if (togo < 0) {
            if (DEBUG)
                System.out.println("goal is to the right");
            if (setpoint.v() < 0) {
                if (DEBUG)
                    System.out.println("We're moving the wrong way (left), so brake.");
                return control(dt, setpoint, goal, togo, 1.0, getScaledD());
            }
            if (setpoint.v() + m_tolerance < lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is below the goal path, so push right.");
                return control(dt, setpoint, goal, togo, 1.0, maxA);
            }
            if (setpoint.v() - m_tolerance < lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is within tolerance of the goal path.");
                return goalPath(dt, setpoint, goal, togo, lerp.a());
            }
            if (DEBUG)
                System.out.println("Setpoint is above the goal path, so brake.");
            return control(dt, setpoint, goal, togo, -1.0, getScaledD());
        } else {
            if (DEBUG)
                System.out.println("goal is to the left");
            if (setpoint.v() > 0) {
                if (DEBUG)
                    System.out.println("We're moving the wrong way (right), so brake.");
                return control(dt, setpoint, goal, togo, -1.0, getScaledD());
            }
            if (setpoint.v() - m_tolerance > lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is above the goal path, so push left.");
                return control(dt, setpoint, goal, togo, -1.0, maxA);
            }
            if (setpoint.v() + m_tolerance > lerp.v()) {
                if (DEBUG)
                    System.out.println("Setpoint is within tolerance of the goal path.");
                return goalPath(dt, setpoint, goal, togo, lerp.a());
            }
            if (DEBUG)
                System.out.println("Setpoint is below the goal path, so brake.");

            return control(dt, setpoint, goal, togo, 1.0, getScaledD());
        }
    }

    @Override
    public ProfileR1 scale(double s) {
        return new CompleteProfileR1(m_log,
                m_maxV, m_maxAUnscaled, m_maxDUnscaled, m_stallAUnscaled,
                m_takeoffJ, m_landingJ, s, m_tolerance);
    }

    private double getScaledA() {
        return m_scale * m_maxAUnscaled;
    }

    private double getScaledD() {
        return m_scale * m_maxDUnscaled;
    }

    private double getScaledStall() {
        return m_scale * m_stallAUnscaled;
    }

    ///////////////////////////////////////

    private ControlR1 control(
            double dt,
            ControlR1 setpoint,
            ModelR1 goal,
            double togo,
            double direction,
            double a) {
        double nextX = togo + setpoint.v() * dt + direction * 0.5 * a * dt * dt;
        double nextV = setpoint.v() + direction * a * dt;
        ControlR1 nextLerp = m_byDistance.get(nextX);
        if (direction * nextV > direction * nextLerp.v()) {
            // The next step spans the goal path, so use the goal path.
            return new ControlR1(goal.x() + nextLerp.x(), nextLerp.v(), nextLerp.a());
        }
        // Setpoint is still far from the goal path, so proceed.
        return new ControlR1(goal.x() + nextX, nextV, direction * a);
    }

    /**
     * Get the next goal-path control near the setpoint.
     */
    private ControlR1 goalPath(
            double dt,
            ControlR1 setpoint,
            ModelR1 goal,
            double togo,
            double accel) {
        double nextX = togo + setpoint.v() * dt + 0.5 * accel * dt * dt;
        ControlR1 nextLerp = m_byDistance.get(nextX);
        return new ControlR1(goal.x() + nextLerp.x(), nextLerp.v(), nextLerp.a());
    }

    /**
     * Acceleration limited by the current limit, back-EMF, and the takeoff jerk
     * limit.
     * Returns a positive number.
     */
    double accel(double dt, ControlR1 setpoint) {
        double speedFraction = Math100.limit(Math.abs(setpoint.v()) / m_maxV, 0, 1);
        double backEmfLimit = 1 - speedFraction;
        double backEmfLimitedAcceleration = backEmfLimit * getScaledStall();
        double currentLimitedAcceleration = getScaledA();
        double jerkLimitedAcceleration = Math.abs(setpoint.a()) + m_takeoffJ * dt;

        if (DEBUG) {
            System.out.printf("fraction %5.2f backEmfLimited %5.2f currentLimited %5.2f jerklimited %5.2f\n",
                    speedFraction, backEmfLimitedAcceleration, currentLimitedAcceleration,
                    jerkLimitedAcceleration);
        }

        return Math.min(Math.min(backEmfLimitedAcceleration, currentLimitedAcceleration), jerkLimitedAcceleration);
    }

    /**
     * Put the control and its mirror on the other side of the goal
     */
    private void put(double t, ControlR1 c) {
        // t is just for debug
        if (DEBUG) {
            System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
        }
        if (DEBUG) {
            System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, -c.x(), -c.v(), -c.a());
        }
        m_byDistance.put(c.x(), c);
        m_byDistance.put(-c.x(), c.mult(-1.0));
    }

    /**
     * This is for the "goal path" which is always slowing down, so use the max
     * decel. The jerk limit affects the "landing".
     */
    private double jerkLimitedAccel(ControlR1 control) {
        if (m_landingJ < 1e-6) {
            // zero endJ means no jerk limit
            return -getScaledD();
        }
        return Math.max(-getScaledD(), control.a() - m_landingJ * DT);
    }
}
