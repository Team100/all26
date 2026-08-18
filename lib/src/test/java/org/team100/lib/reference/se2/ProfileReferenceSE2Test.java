package org.team100.lib.reference.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.profile.se2.HolonomicProfile;
import org.team100.lib.profile.se2.HolonomicProfileFactory;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProfileReferenceSE2Test implements Timeless {
    private static final double DELTA = 0.001;
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testSimple() {
        ModelSE2 measurement = new ModelSE2(new Pose2d(0, 0, Rotation2d.kZero));
        ModelSE2 goal = new ModelSE2(new Pose2d(1, 0, Rotation2d.kZero));
        HolonomicProfile hp = HolonomicProfileFactory.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        ProfileReferenceSE2 r = new ProfileReferenceSE2(logger, hp, "test");
        r.setGoal(goal);
        r.initialize(measurement);
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.02, n.velocity().x(), DELTA);
            assertEquals(0, n.pose().getX(), DELTA);
        }
        // no time step, nothing changes
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.02, n.velocity().x(), DELTA);
            // x is very small but not zero
            assertEquals(0.0002, n.pose().getX(), 0.00001);
        }
        // stepping time gets the next references
        stepTime();
        {
            ModelSE2 c = r.current();
            assertEquals(0.02, c.velocity().x(), DELTA);
            assertEquals(0, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0.04, n.velocity().x(), DELTA);
            assertEquals(0.00078, n.pose().getX(), DELTA);
        }
        // way in the future, we're at the end.
        for (int i = 0; i < 500; ++i) {
            stepTime();
        }
        {
            ModelSE2 c = r.current();
            assertEquals(0, c.velocity().x(), DELTA);
            assertEquals(1, c.pose().getX(), DELTA);
            ControlSE2 n = r.next();
            assertEquals(0, n.velocity().x(), DELTA);
            assertEquals(1, n.pose().getX(), DELTA);
        }
    }

}
