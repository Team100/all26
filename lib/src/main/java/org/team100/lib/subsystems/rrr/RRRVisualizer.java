package org.team100.lib.subsystems.rrr;

import java.util.function.Supplier;

import org.team100.lib.geometry.rrr.RRRConfig;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * Use the glass "mechanism" display to show the arm position.
 * 
 * "x" axis points "up".
 */
public class RRRVisualizer {
    private static final int WIDTH = 10;
    private static final Color8Bit COLOR = new Color8Bit(Color.ORANGE_RED);
    private static final Color8Bit COLOR2 = new Color8Bit(Color.WHITE);
    private static final double SCALE = 150;

    private final RRRArm m_arm;
    private final MechanismLigament2d m_l1;
    private final MechanismLigament2d m_l2;
    private final MechanismLigament2d m_l3;
    private final MechanismRoot2d m_cross;
    // making this mutable makes it easier to sequence construction
    private Supplier<Translation2d> m_t;

    public RRRVisualizer(RRRArm arm) {
        m_arm = arm;
        Mechanism2d view = new Mechanism2d(200, 150);
        MechanismRoot2d root = view.getRoot("root", 100, 25);
        m_l1 = new MechanismLigament2d(
                "l1", SCALE * m_arm.kinematics().l1, 90, WIDTH, COLOR);
        m_l2 = new MechanismLigament2d(
                "l2", SCALE * m_arm.kinematics().l2, 0, WIDTH, COLOR);
        m_l3 = new MechanismLigament2d(
                "l3", SCALE * m_arm.kinematics().l3, 0, WIDTH, COLOR);
        root.append(m_l1);
        m_l1.append(m_l2);
        m_l2.append(m_l3);
        m_cross = view.getRoot("pose", 100, 25);
        m_cross.append(new MechanismLigament2d("cross1", 10, 0, 1, COLOR2));
        m_cross.append(new MechanismLigament2d("cross2", 10, 90, 1, COLOR2));
        m_cross.append(new MechanismLigament2d("cross3", 10, 180, 1, COLOR2));
        m_cross.append(new MechanismLigament2d("cross4", 10, 270, 1, COLOR2));

        SmartDashboard.putData("View", view);
    }

    public void setT(Supplier<Translation2d> t) {
        m_t = t;
    }

    public void periodic() {
        RRRConfig q = m_arm.getConfig();
        m_l1.setAngle(90 + Math.toDegrees(q.q1()));
        m_l2.setAngle(Math.toDegrees(q.q2()));
        m_l3.setAngle(Math.toDegrees(q.q3()));

        if (m_t != null) {
            Translation2d t = m_t.get();
            if (t != null)
                m_cross.setPosition(100 - SCALE * t.getY(), 25 + SCALE * t.getX());
        }
    }
}
