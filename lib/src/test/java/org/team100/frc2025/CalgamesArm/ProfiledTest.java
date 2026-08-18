package org.team100.frc2025.CalgamesArm;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.kinematics.prr.PRRKinematics;
import org.team100.lib.kinematics.prr.PRRKinematics.Solver;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.TrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProfiledTest {
    private static final boolean DEBUG = false;
    private static final double DT = 0.02;

    /**
     * This shows that home-to-pick works fine as a profile, because it's almost
     * entirely shoulder motion, nearly circular. Also, since it's in the "reach
     * down" regime of the mechanism, a config profile for this path is much easier
     * to make work.
     * 
     * For charts, see
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=1920529239#gid=1920529239
     */
    @Test
    void homeToPick() {

        PRRKinematics k = new PRRKinematics(0.5, 0.343, Solver.ANALYTIC);

        // home position
        PRRConfig start = new PRRConfig(0, 0, 0);
        // floor pick position
        PRRConfig goal = new PRRConfig(0, -3 * Math.PI / 4, Math.PI / 4);

        ModelR1 g1 = new ModelR1(goal.q1(), 0);
        ModelR1 g2 = new ModelR1(goal.q2(), 0);
        ModelR1 g3 = new ModelR1(goal.q3(), 0);
        ProfileR1 p1 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p2 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p3 = new TrapezoidProfileR1(1, 1, 0.05);

        ControlR1 i1 = new ControlR1(start.q1(), 0);
        ControlR1 i2 = new ControlR1(start.q2(), 0);
        ControlR1 i3 = new ControlR1(start.q3(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            PRRConfig c = new PRRConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.q1(),
                        c.q2(), c.q3(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }

    }

    /**
     * This shows why uncoordinated profiles are a bad choice for scoring: the
     * profile goes out too soon, and would end up hitting the scoring posts from
     * the bottom on the way up.
     * 
     * For charts, see
     * 
     * https://docs.google.com/spreadsheets/d/1yo5gU4NwVDUP8XaGb-7jNOtN6A_7cpX3DWRTfwgKym0/edit?gid=1974165792#gid=1974165792
     */
    @Test
    void homeToL4() {

        PRRKinematics k = new PRRKinematics(0.5, 0.343, Solver.ANALYTIC);

        // home position
        PRRConfig start = new PRRConfig(0, 0, 0);

        Pose2d pL4 = new Pose2d(1.9, 0.5, new Rotation2d(150));
        // floor pick position
        PRRConfig goal = k.inverse(pL4);

        ModelR1 g1 = new ModelR1(goal.q1(), 0);
        ModelR1 g2 = new ModelR1(goal.q2(), 0);
        ModelR1 g3 = new ModelR1(goal.q3(), 0);
        ProfileR1 p1 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p2 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p3 = new TrapezoidProfileR1(1, 1, 0.05);

        ControlR1 i1 = new ControlR1(start.q1(), 0);
        ControlR1 i2 = new ControlR1(start.q2(), 0);
        ControlR1 i3 = new ControlR1(start.q3(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            PRRConfig c = new PRRConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.q1(),
                        c.q2(), c.q3(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }
    }

    /**
     * Profile down is also not safe.
     */
    @Test
    void l4ToHome() {

        PRRKinematics k = new PRRKinematics(0.5, 0.343, Solver.ANALYTIC);
        Pose2d pL4 = new Pose2d(1.9, 0.5, new Rotation2d(150));

        // home position
        PRRConfig start = k.inverse(pL4);

        // floor pick position
        PRRConfig goal = new PRRConfig(0, 0, 0);

        ModelR1 g1 = new ModelR1(goal.q1(), 0);
        ModelR1 g2 = new ModelR1(goal.q2(), 0);
        ModelR1 g3 = new ModelR1(goal.q3(), 0);
        ProfileR1 p1 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p2 = new TrapezoidProfileR1(1, 1, 0.05);
        ProfileR1 p3 = new TrapezoidProfileR1(1, 1, 0.05);

        ControlR1 i1 = new ControlR1(start.q1(), 0);
        ControlR1 i2 = new ControlR1(start.q2(), 0);
        ControlR1 i3 = new ControlR1(start.q3(), 0);

        double eta1 = p1.simulateForETA(DT, i1, g1);
        double eta2 = p1.simulateForETA(DT, i2, g2);
        double eta3 = p1.simulateForETA(DT, i3, g3);

        double eta = Math.max(eta1, Math.max(eta2, eta3));

        if (DEBUG)
            System.out.println("t, x, y, r, q1, q2, q3, q1dot, q2dot, q3dot, q1ddot, q2ddot, q3ddot");
        for (double tt = 0; tt < eta; tt += DT) {
            i1 = p1.calculate(DT, i1, g1);
            i2 = p2.calculate(DT, i2, g2);
            i3 = p3.calculate(DT, i3, g3);
            PRRConfig c = new PRRConfig(i1.x(), i2.x(), i3.x());
            Pose2d p = k.forward(c);

            if (DEBUG) {
                System.out.printf(
                        "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n",
                        tt, p.getX(), p.getY(), p.getRotation().getRadians(), c.q1(), c.q2(),
                        c.q3(), i1.v(),
                        i2.v(), i3.v(), i1.a(), i2.a(), i3.a());
            }
        }
    }

}
