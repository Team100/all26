package org.team100.lib.visualization;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.kinematics.five_bar.JointPositions;
import org.team100.lib.kinematics.five_bar.Point;
import org.team100.lib.kinematics.five_bar.Scenario;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Uses the Mechanism2d widget to visualize the five-bar.
 * 
 * The widget uses a sequence of links. The length and parent-relative angle of
 * each link can be adjusted.
 * 
 * The forward kinematics of the five-bar produces cartesian points, so we have
 * to extract the link-sequence angles and lengths from them.
 * 
 * The root link ("a5") is not rendered.
 * 
 * The orientation of this visualization matches the fivebar training apparatus,
 * wherein the "y" axis is pointing towards the user, so "down" in the viz, and
 * the "x" axis is therefore pointing to the left. The working area is entirely
 * "below" the origin (i.e. positive y).
 * 
 * Important! Fivebar kinematics are indeterminate: for any feasible root
 * angles (except one), there are two possible end-effector positions.
 */
public class FiveBarVisualization {
    private static final boolean DEBUG = false;

    private static final double WIDTH = 10;
    private static final Color8Bit ORANGE = new Color8Bit(Color.kOrangeRed);
    private final double SCALE;
    private final Supplier<Optional<JointPositions>> m_q;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_p1;
    private final MechanismLigament2d m_a1;
    private final MechanismLigament2d m_a2;
    private final MechanismLigament2d m_a3;
    private final MechanismLigament2d m_a4;

    public FiveBarVisualization(
            Scenario scenario,
            Supplier<Optional<JointPositions>> q) {
        m_q = q;
        // scale to link length
        SCALE = 75 / (scenario.a1 + scenario.a2);
        m_view = new Mechanism2d(100, 100);
        Point p1 = scenario.P1();
        Point p5 = scenario.P5();
        // the midpoint of a5
        Point a5Mid = p1.plus(p5.minus(p1).times(0.5));
        m_p1 = m_view.getRoot("root", 50 + SCALE * a5Mid.x(), 75 + SCALE * a5Mid.y());
        m_a1 = new MechanismLigament2d("a1", 0, 0, WIDTH, ORANGE);
        m_p1.append(m_a1);
        m_a2 = new MechanismLigament2d("a2", 0, 0, WIDTH, ORANGE);
        m_a1.append(m_a2);
        m_a3 = new MechanismLigament2d("a3", 0, 0, WIDTH, ORANGE);
        m_a2.append(m_a3);
        m_a4 = new MechanismLigament2d("a4", 0, 0, WIDTH, ORANGE);
        m_a3.append(m_a4);
        SmartDashboard.putData("View", m_view);
    }

    public void periodic() {
        Optional<JointPositions> optQ = m_q.get();
        if (optQ.isEmpty())
            return;
        JointPositions q = optQ.get();
        List<Point> p = links(q);
        if (DEBUG) {
            System.out.printf("FiveBarVisualization q %s %s %s %s %s\n",
                    q.P1(), q.P2(), q.P3(), q.P4(), q.P5());
            System.out.printf("FiveBarVisualization links %s %s %s %s\n",
                    p.get(0), p.get(1), p.get(2), p.get(3));
            System.out.printf("FiveBarVisualization norms %f %f %f %f\n",
                    p.get(0).norm(), p.get(1).norm(), p.get(2).norm(), p.get(3).norm());
        }
        m_a1.setLength(SCALE * p.get(0).norm());
        m_a2.setLength(SCALE * p.get(1).norm());
        m_a3.setLength(SCALE * p.get(2).norm());
        m_a4.setLength(SCALE * p.get(3).norm());
        // angle references +x which is to the left
        m_a1.setAngle(Math.toDegrees(p.get(0).angle().orElseThrow()) + 180);
        m_a2.setAngle(Math.toDegrees(p.get(1).angle().orElseThrow()));
        m_a3.setAngle(Math.toDegrees(p.get(2).angle().orElseThrow()));
        m_a4.setAngle(Math.toDegrees(p.get(3).angle().orElseThrow()));
    }

    static List<Point> links(JointPositions q) {
        Point a1 = q.P2().minus(q.P1());
        Point a2 = q.P3().minus(q.P2());
        Point a3 = q.P4().minus(q.P3());
        Point a4 = q.P5().minus(q.P4());
        return List.of(
                a1,
                a2.rotateBy(-1.0 * a1.angle().orElseThrow()),
                a3.rotateBy(-1.0 * a2.angle().orElseThrow()),
                a4.rotateBy(-1.0 * a3.angle().orElseThrow()));
    }

}
