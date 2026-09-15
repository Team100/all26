package org.team100.lib.gtsam.pose_estimator;

import org.junit.jupiter.api.Test;

import gtsam.Key;
import gtsam.Pose2;
import gtsam.Vector1;
import gtsam.Vector3;
import gtsam.noiseModel.Diagonal;

public class BetweenGyroTest {
    @Test
    void testSimple() throws Throwable {
        Solver solver = new Solver(100000);
        BetweenGyro b = new BetweenGyro(solver);
        Prior p = new Prior(solver);

        // Variables
        solver.addVariable(Key.X(0), 0, new Pose2());
        solver.addVariable(Key.X(20000), 20000, new Pose2());
        solver.addVariable(Key.B(0), 0, 0);
        solver.addVariable(Key.B(20000), 0, 0);

        // Priors
        p.add(Key.X(0), new Pose2(), Diagonal.Sigmas(new Vector3(10, 10, 10)));
        p.add(Key.X(20000), new Pose2(), Diagonal.Sigmas(new Vector3(10, 10, 10)));
        p.add(Key.B(0), 0, Diagonal.Sigmas(new Vector1(1)));
        p.add(Key.B(20000), 0, Diagonal.Sigmas(new Vector1(1)));

        // Gyro factors
        b.add(0, 0); // just logs the zero
        b.add(20000, 1); // actually adds the factors

        solver.update();

        Pose2 p0 = solver.mean_pose2(Key.X(0));
        Pose2 p1 = solver.mean_pose2(Key.X(20000));
        double b0 = solver.mean_double(Key.B(0));

        System.out.printf("p0 %f %f %f\n", p0.x(), p0.y(), p0.theta());
        System.out.printf("p1 %f %f %f\n", p1.x(), p1.y(), p1.theta());
        System.out.printf("b0 %f\n", b0);
    }

}
