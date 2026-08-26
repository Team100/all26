package org.team100.lib.profile.r1;

import org.team100.lib.optimization.Bisection1d;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;

/**
 * This profile takes incremental steps from the setpoint towards the goal.
 * 
 * Use the ETA to coordinate multiple dimensions.
 * 
 * NOTE: these profiles don't know anything about angle wrapping.
 */
public interface ProfileR1 {
    public static final int MAX_ETA = 10;
    static final boolean DEBUG = false;

    /**
     * Return the control for dt in the future. The setpoint is a ControlR1 so that
     * we can regulate jerk.
     */
    ControlR1 calculate(double dt, ControlR1 setpoint, StateR1 goal);

    /**
     * Find ETA by simply running the whole thing to the end.
     * You should use a coarse dt so that this doesn't take too long.
     * It's ok for the answer to be a little bit wrong.
     * 
     * The reasons we use a simulation approach are that all our profiles are short
     * so it's not that hard to do, some profiles can be hard to reason about
     * without simulating anyway, and it's guaranteed to work.
     */
    default double simulateForETA(double dt, ControlR1 initial, StateR1 goal) {
        double t = 0;
        StateR1 sample = initial.model();
        while (!sample.near(goal, 0.01)) {
            ControlR1 c = calculate(dt, sample.control(), goal);
            sample = c.model();
            t += dt;
            if (t > MAX_ETA)
                return Double.POSITIVE_INFINITY;
        }
        return t;
    }

    /**
     * Return a new profile scaled by s. The choice of what the parameter actually
     * does is up to the implementation. A good choice would be to scale
     * acceleration, since that's what goes against the power budget.
     */
    ProfileR1 scale(double s);

    /**
     * Find the scale factor that makes the profile complete in the specified time
     * (ETA).
     * 
     * It never returns s > 1, and it also never scales more than 10X, i.e. never
     * returns s < 0.01.
     * 
     * It is very approximate, in order to not run too long. It's very primitive.
     */
    default double solve(
            double dt,
            ControlR1 i,
            StateR1 g,
            double goalETA,
            double etaTolerance) {
        final double minS = 0.01;
        final double maxS = 1.0;
        double ss = Bisection1d.findRoot(
                s -> scale(s).simulateForETA(dt, i, g) - goalETA,
                minS,
                scale(minS).simulateForETA(dt, i, g) - goalETA,
                maxS,
                scale(maxS).simulateForETA(dt, i, g) - goalETA,
                etaTolerance,
                100);
        if (DEBUG) {
            System.out.printf("s %5.2f\n", ss);
        }
        return ss;
    }

}
