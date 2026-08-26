package org.team100.lib.controller.r1;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.StateR1;

public class PIDFeedbackTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    public void testCalculate() {
        FeedbackR1 c = new PIDFeedback(logger, 1, 0, 0, false, 0.05, 1);

        // zero error, zero feedback
        double u = c.calculate(new StateR1(), new StateR1());
        assertEquals(0, u, DELTA);

        // position error -> u
        u = c.calculate(new StateR1(0, 0), new StateR1(1, 0));
        assertEquals(1, u, DELTA);

        // this controller ignores velocity
        u = c.calculate(new StateR1(0, 0), new StateR1(0, 1));
        assertEquals(0, u, DELTA);
    }
}
