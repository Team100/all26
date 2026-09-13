package org.team100.lib.kinematics.rrr_se2;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rrr.RRRConfig;

public class RRRFeasibilityTest {
    @Test
    void test0() {
        RRRKinematicsPoE k = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        RRRFeasibility f = new RRRFeasibility(k,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        assertTrue(f.qRange(new RRRConfig(0, 0, 0)));
        assertFalse(f.qRange(new RRRConfig(0, 0, 2)));
        assertTrue(f.xRange(new RRRConfig(0, 0, 0)));
        assertFalse(f.xRange(new RRRConfig(2, 0, 0)));
    }

}
