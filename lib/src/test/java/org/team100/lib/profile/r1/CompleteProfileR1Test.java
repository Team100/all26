package org.team100.lib.profile.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * see
 * https://docs.google.com/spreadsheets/d/1JdKViVSTEMZ0dRS8broub4P-f0eA6STRHHzoV0U4N5M/edit?gid=2097479642#gid=2097479642
 */
public class CompleteProfileR1Test implements Timeless{
    private static final boolean DEBUG = false;
    private static final double DT = 0.02;
    private static final double DELTA = 0.001;
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    /** Dump the sliding mode curve */
    @Test
    void testMode() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 2, 6, 10, 40, 50, 50, 0.001);
        if (DEBUG) {
            for (double x = -10; x < 10; x += 0.01) {
                ControlR1 sample = p.m_byDistance.get(x);
                System.out.printf("%12.4f %12.4f %12.4f\n", sample.x(), sample.v(), sample.a());
            }
        }
    }

    @Test
    void testInterpolation() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 2, 6, 10, 40, 50, 50, 0.001);
        ControlR1 c = p.m_byDistance.get(-500.0);
        // we get back the x coord we provided
        assertEquals(-500, c.x(), DELTA);
        // v is always maxv
        assertEquals(2, c.v(), DELTA);
        // a is always zero
        assertEquals(0, c.a(), DELTA);
    }

    @Test
    void testFastAccelSlowDecel() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 5, 12, 5, 50, 50, 50, 0.001);
        final ModelR1 goal = new ModelR1(2, 0);
        ControlR1 c = new ControlR1();
        double t = 0;
        if (DEBUG)
            System.out.println("t, x, v, a");
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("%.3f, %.3f, %.3f, %.3f\n",
                        t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c, goal);
            t += DT;
        }
    }

    @Test
    void testSlowAccelFastDecel() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 5, 5, 12, 50, 50, 50, 0.001);
        final ModelR1 goal = new ModelR1(2, 0);
        ControlR1 c = new ControlR1();
        double t = 0;
        if (DEBUG)
            System.out.println("t, x, v, a");
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("%.3f, %.3f, %.3f, %.3f\n",
                        t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c, goal);
            t += DT;
        }
    }

    @Test
    void testSimpleBackward() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 3, 8, 12, 15, 50, 50, 0.001);
        final ModelR1 goal = new ModelR1(-2, 0);
        ControlR1 c = new ControlR1();
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c, goal);
            t += DT;
        }
    }

    @Test
    void testMovingEntry() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 2, 6, 10, 30, 50, 50, 0.001);
        final ModelR1 goal = new ModelR1(1, 0);
        ControlR1 c = new ControlR1(0, -1);
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c, goal);
            t += DT;
        }
    }

    @Test
    void testUTurn() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 3, 8, 12, 15, 50, 50, 0.001);
        final ModelR1 goal = new ModelR1(0, 0);
        // to the left and moving to the left
        ControlR1 c = new ControlR1(-2, -2);
        double t = 0;
        for (int i = 0; i < 100; ++i) {
            if (DEBUG)
                System.out.printf("%12.4f %12.4f %12.4f %12.4f\n", t, c.x(), c.v(), c.a());
            c = p.calculate(DT, c, goal);
            t += DT;
        }
    }

    /** Moving goals are not allowed. */
    @Test
    void testMovingGoal() {
        CompleteProfileR1 p = new CompleteProfileR1(logger, 2, 6, 10, 30, 50, 50, 0.01);
        assertThrows(IllegalArgumentException.class,
                () -> p.calculate(0.02, new ControlR1(), new ModelR1(1, 1)));
    }

    /** How does interpolation work? */
    @Test
    void testKeyInterpolation1() {
        InterpolatingDoubleTreeMap m = new InterpolatingDoubleTreeMap();
        m.put(-1.0, 1.0);
        m.put(1.0, -1.0);
        // it takes the endpoint forever
        assertEquals(1, m.get(-3.0), DELTA);
        assertEquals(1, m.get(-2.0), DELTA);
        assertEquals(0, m.get(0.0), DELTA);
        assertEquals(-1, m.get(2.0), DELTA);
        assertEquals(-1, m.get(3.0), DELTA);
    }

    /** What if one of the points is really far away? */
    @Test
    void testKeyInterpolation2() {
        InterpolatingTreeMap<Double, ControlR1> m = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                ControlR1::interpolate);
        // without this far-away point, the interpolator returns the endpoint at -1.
        m.put(-384400000.0, new ControlR1(-384400000, 1, 0));
        m.put(-1.0, new ControlR1(-1, 1, 0));
        m.put(1.0, new ControlR1(1, -1, 0));
        m.put(384400000.0, new ControlR1(384400000, -1, 0));
        // it takes the endpoint forever
        assertEquals(-3, m.get(-3.0).x(), DELTA);
        assertEquals(1, m.get(-3.0).v(), DELTA);
        assertEquals(-2, m.get(-2.0).x(), DELTA);
        assertEquals(0, m.get(0.0).x(), DELTA);
        assertEquals(2, m.get(2.0).x(), DELTA);
        assertEquals(3, m.get(3.0).x(), DELTA);
    }
}
