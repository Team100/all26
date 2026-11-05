package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.uncertainty.IsotropicNoiseSE2;
import org.team100.lib.uncertainty.VariableR1;

class LocalizerEstimateTest implements Timeless {

    // this is a 0.5 m square.
    SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest();

    @Test
    void testInterp0a() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        VariableR1 gyroBias = VariableR1.fromVariance(0, 0.001);
        LocalizerEstimate estimate = new LocalizerEstimate(s0, n0, gyroBias);
        assertEquals(0, estimate.state().theta().x(), 0.001);
        assertEquals(0, estimate.state().theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/4.
        // Each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        // current speed is 1 rad/s
        LocalizerEstimate estimate1 = new LocalizerEstimate(s1, n1, VariableR1.fromVariance(0, 1));
        assertEquals(1, estimate1.state().theta().x(), 0.001);
        assertEquals(1, estimate1.state().theta().v(), 0.001);
    }

    @Test
    void test1() {
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        LocalizerEstimate estimate = new LocalizerEstimate(s0, n0, VariableR1.fromVariance(0, 1));
        assertEquals(0, estimate.state().theta().x(), 0.001);
        assertEquals(0, estimate.state().theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/4.
        // Each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        LocalizerEstimate estimate2 = new LocalizerEstimate(s1, n1, VariableR1.fromVariance(0, 1));

        assertEquals(1, estimate2.state().theta().x(), 0.001);
        assertEquals(1, estimate2.state().theta().v(), 0.001);
    }

    @Test
    void testInterp0() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);

        VariableR1 gyroBias = VariableR1.fromVariance(0, 0.001);
        LocalizerEstimate estimate0 = new LocalizerEstimate(s0, n0, gyroBias);

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        // current speed is 1 rad/s
        VariableR1 gyroBias2 = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate1 = new LocalizerEstimate(s1, n1, gyroBias2);

        {
            // t=0 should return r0. note "t" is not time
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 0.0);
            assertEquals(0, lerp.state().theta().x(), 0.001);
            assertEquals(0, lerp.state().theta().v(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 1.0);
            assertEquals(1, lerp.state().theta().x(), 0.001);
            assertEquals(1, lerp.state().theta().v(), 0.001);
        }
        {
            // t=0.5 is just halfway there.
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 0.5);
            assertEquals(0.5, lerp.state().theta().x(), 0.001);
            assertEquals(0.5, lerp.state().theta().v(), 0.001);
        }
    }

    @Test
    void testInterp1() {
        StateSE2 s0 = new StateSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);

        VariableR1 gyroBias = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate0 = new LocalizerEstimate(s0, n0, gyroBias);

        StateSE2 s1 = new StateSE2(new StateR1(), new StateR1(), new StateR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        // current speed is 1 rad/s
        VariableR1 gyroBias2 = VariableR1.fromVariance(0, 1);
        LocalizerEstimate estimate1 = new LocalizerEstimate(s1, n1, gyroBias2);

        {
            // t=0 should return r0. note "t" is not time
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 0.0);
            assertEquals(0, lerp.state().theta().x(), 0.001);
            assertEquals(0, lerp.state().theta().v(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 1.0);
            assertEquals(1, lerp.state().theta().x(), 0.001);
            assertEquals(1, lerp.state().theta().v(), 0.001);
        }
        {
            // t=0.5
            LocalizerEstimate lerp = estimate0.interpolate(estimate1, 0.5);
            // wheels are assumed to turn immediately to the r1 angles,
            // so this case is the same as the case above.
            assertEquals(0.5, lerp.state().theta().x(), 0.001);
            assertEquals(0.5, lerp.state().theta().v(), 0.001);
        }
    }

}
