package org.team100.lib.kinematics.five_bar;

import static java.lang.Math.PI;
import static java.lang.Math.acos;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import java.util.Optional;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Math100;

/**
 * Kinematics of 2-dof 5-bar planar linkage with one grounded bar.
 * 
 * https://en.wikipedia.org/wiki/Five-bar_linkage
 * 
 * Adapted from http://charm.stanford.edu/ME327/JaredAndSam
 * 
 * Which is itself adapted from "The Pantograph Mk-II: A Haptic Instrument"
 * Hayward, 2005 https://cim.mcgill.ca/~haptic/pub/GC-QW-VH-IROS-05.pdf
 * 
 * See pantograph.png for the coordinates used here.
 */
public class FiveBarKinematics {

    private final DoubleLogger m_log_theta1;
    private final DoubleLogger m_log_theta5;

    public FiveBarKinematics(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_log_theta1 = log.doubleLogger(Level.COMP, "theta 1");
        m_log_theta5 = log.doubleLogger(Level.COMP, "theta 5");
    }

    /**
     * Computes inverse kinematics, after Hayward 2005.
     * 
     * See angles.png for the details.
     * 
     * Important! Fivebar inverse kinematics are indeterminate: there may
     * be up to four solutions (see forward.png). This function always
     * yields the "+ -" or "elbows out" solution.
     * 
     * @param scenario simulation geometry
     * @param x3,y3    position of end effector ("P3" in the diagram), meters
     * @return the angles of the proximal links, or empty if no solution is
     *         possible.
     */
    public Optional<ActuatorAngles> inverse(Scenario scenario, double x3, double y3) {
        try {
            // Distance from P1 to P3.
            double P13 = Math100.isFinite(sqrt((pow(x3, 2)) + (pow(y3, 2))));
            // Distance from P5 to P3.
            double P53 = Math100.isFinite(sqrt((pow((x3 + scenario.a5), 2)) + (pow(y3, 2))));

            // Angle P2-P1-P3
            double alpha1 = Math100.isFinite(acos(
                    ((pow(scenario.a1, 2)) + (pow(P13, 2)) - (pow(scenario.a2, 2))) / (2 * scenario.a1 * P13)));
            // Angle P5-P1-P3
            double beta1 = Math100.isFinite(atan2(y3, -x3));
            // Angle of P1
            double theta1 = PI - alpha1 - beta1;
            m_log_theta1.log(() -> theta1);

            // Angle P1-P5-P3
            double alpha5 = Math100.isFinite(atan2(y3, x3 + scenario.a5));
            // Angle P4-P5-P3
            double beta5 = Math100.isFinite(acos(
                    ((pow(P53, 2)) + (pow(scenario.a4, 2)) - (pow(scenario.a3, 2))) / (2 * P53 * scenario.a4)));
            // Angle of P5
            double theta5 = alpha5 + beta5;
            m_log_theta5.log(() -> theta5);

            return Optional.of(new ActuatorAngles(theta1, theta5));
        } catch (IllegalArgumentException e) {
            return Optional.empty();
        }
    }

    /**
     * Computes forward kinematics.
     * 
     * See pantograph.png for details.
     * 
     * Important! Fivebar forward kinematics are indeterminate: for any root angle
     * pair, there may be zero, one, or two solutions.
     * 
     * This function returns the "outermost" of the two solutions, so it
     * can never reach the baseline.
     * 
     * @param scenario simulation geometry
     * @param theta1       the angle between the x axis and a1
     * @param theta5       the angle between the x axis and a4
     * @returns the five joint positions, and also the center of the hypotenuse, or
     *          empty if no solution is possible.
     */
    public Optional<JointPositions> forward(Scenario scenario, double theta1, double theta5) {
        try {
            // P1 is fixed by the scenario (usually the origin).
            Point P1 = scenario.P1();

            // P2 is simply the angle, t1, at the radius, a1, from P1.
            double x2 = scenario.a1 * cos(theta1);
            double y2 = scenario.a1 * sin(theta1);
            Point P2 = new Point(x2, y2);

            // P4 is simply the angle, t5, at the radius, a4, from P5.
            double x4 = scenario.a4 * cos(theta5) - scenario.a5;
            double y4 = scenario.a4 * sin(theta5);
            Point P4 = new Point(x4, y4);

            double P4P2 = P4.distance(P2);
            double P2Ph = Math100.isFinite((pow(scenario.a2, 2) - pow(scenario.a3, 2) + pow(P4P2, 2)) / (2 * P4P2));
            // Midpoint of the base of the P2-P4-P3 triangle.
            Point Ph = P2.plus((P4.minus(P2)).times((P2Ph / P4P2)));
            // Height of the P2-P4-P3 triangle
            double P3Ph = Math100.isFinite(sqrt(pow(scenario.a2, 2) - pow(P2Ph, 2)));

            // The end-effector position.
            double x3 = Math100.isFinite(Ph.x() + (P3Ph / P2.distance(P4)) * (y4 - y2));
            double y3 = Math100.isFinite(Ph.y() - (P3Ph / P2.distance(P4)) * (x4 - x2));
            Point P3 = new Point(x3, y3);

            // P5 is fixed by the scenario.
            Point P5 = scenario.P5();

            return Optional.of(new JointPositions(P1, P2, P3, P4, P5, Ph));
        } catch (IllegalArgumentException e) {
            return Optional.empty();
        }
    }
}