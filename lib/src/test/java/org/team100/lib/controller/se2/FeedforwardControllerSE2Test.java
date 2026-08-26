package org.team100.lib.controller.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.state.VelocityControlSE2;

public class FeedforwardControllerSE2Test {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testMotionless() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocityControlSE2 v = c.calculate(
                new StateSE2(
                        new StateR1(0, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new StateSE2(
                        new StateR1(0, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new ControlSE2(
                        new ControlR1(0, 0),
                        new ControlR1(0, 0),
                        new ControlR1(0, 0)));
        assertEquals(0, v.x().v(), DELTA);
        assertEquals(0, v.y().v(), DELTA);
        assertEquals(0, v.theta().v(), DELTA);
        assertTrue(c.atReference());
    }

    @Test
    void testNotAtReference() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocityControlSE2 v = c.calculate(
                new StateSE2(
                        new StateR1(1, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new StateSE2(
                        new StateR1(0, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new ControlSE2(
                        new ControlR1(0, 0),
                        new ControlR1(0, 0),
                        new ControlR1(0, 0)));
        assertEquals(0, v.x().v(), DELTA);
        assertEquals(0, v.y().v(), DELTA);
        assertEquals(0, v.theta().v(), DELTA);
        assertFalse(c.atReference());
    }

    @Test
    void testFeedforward() {
        FeedforwardControllerSE2 c = new FeedforwardControllerSE2(logger, 0.01, 0.01, 0.01, 0.01);
        assertFalse(c.atReference());
        VelocityControlSE2 v = c.calculate(
                new StateSE2(
                        new StateR1(0, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new StateSE2(
                        new StateR1(0, 0),
                        new StateR1(0, 0),
                        new StateR1(0, 0)),
                new ControlSE2(
                        new ControlR1(0, 1),
                        new ControlR1(0, 0),
                        new ControlR1(0, 0)));
        assertEquals(1, v.x().v(), DELTA);
        assertEquals(0, v.y().v(), DELTA);
        assertEquals(0, v.theta().v(), DELTA);
        assertTrue(c.atReference());
    }
}
