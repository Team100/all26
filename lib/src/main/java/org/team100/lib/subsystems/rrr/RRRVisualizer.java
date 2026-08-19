package org.team100.lib.subsystems.rrr;

import org.team100.lib.geometry.rrr.RRRConfig;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/** Use the glass "mechanism" display to show the arm position. */
public class RRRVisualizer {
    private static final double SCALE = 100;
    private final RRRArm m_arm;
    private final MechanismLigament2d m_l1;
    private final MechanismLigament2d m_l2;
    private final MechanismLigament2d m_l3;

    public RRRVisualizer(RRRArm arm) {
        m_arm = arm;
        Mechanism2d view = new Mechanism2d(200, 200);
        MechanismRoot2d root = view.getRoot("root", 100, 100);
        m_l1 = new MechanismLigament2d(
                "l1", SCALE * m_arm.m_kinematics.l1, 0,
                5, new Color8Bit(Color.ORANGE_RED));
        m_l2 = new MechanismLigament2d(
                "l2", SCALE * m_arm.m_kinematics.l2, 0,
                5, new Color8Bit(Color.ORANGE_RED));
        m_l3 = new MechanismLigament2d(
                "l3", SCALE * m_arm.m_kinematics.l3, 0,
                5, new Color8Bit(Color.ORANGE_RED));
        root.append(m_l1);
        m_l1.append(m_l2);
        m_l2.append(m_l3);
        SmartDashboard.putData("View", view);
    }

    public void periodic() {
        RRRConfig q = m_arm.getConfig();
        m_l1.setAngle(Math.toDegrees(q.q1()));
        m_l2.setAngle(Math.toDegrees(q.q2()));
        m_l3.setAngle(Math.toDegrees(q.q3()));
    }
}
