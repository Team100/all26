package org.team100.lib.gtsam.pose_estimator;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import gtsam.Key;
import gtsam.PriorFactor;
import gtsam.Vector1;
import gtsam.noiseModel.Diagonal;

public class SolverTest {

    @Test
    void testAddFactor() throws Throwable {
        Solver solver = new Solver(100000);
        // The key does not exist.
        Key key = new Key(1);
        assertFalse(solver.add(PriorFactor.PriorFactorDouble(
                key, 0, Diagonal.Sigmas(new Vector1(1)))));
        solver.addVariable(key, 1, 1);
        // Now the key exists.
        assertTrue(solver.add(PriorFactor.PriorFactorDouble(
                key, 0, Diagonal.Sigmas(new Vector1(1)))));
    }
}
