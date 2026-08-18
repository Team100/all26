package org.team100.frc2025.CalgamesArm;


import org.team100.lib.geometry.prr.PRRConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CalgamesViz implements Runnable {
    private static final double SCALE = 100;
    private static final Translation2d ORIGIN = new Translation2d(50, 0);
    private final CalgamesMech m_mech;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_hand;

    public CalgamesViz(CalgamesMech mech) {
        m_mech = mech;
        m_view = new Mechanism2d(SCALE, SCALE);
        m_root = m_view.getRoot("root", ORIGIN.getX(), ORIGIN.getY());
        PRRConfig q = m_mech.getConfig();
        m_elevator = new MechanismLigament2d(
                "elevator",
                shoulderHeight(q),
                90); // elevator is always at 90
        m_root.append(m_elevator);
        m_arm = new MechanismLigament2d(
                "arm",
                SCALE * m_mech.getArmLength(), // constant length
                shoulderAngle(q));
        m_elevator.append(m_arm);
        m_hand = new MechanismLigament2d(
                "hand",
                SCALE * m_mech.getHandLength(), // constant length
                wristAngle(q));
        m_arm.append(m_hand);
        SmartDashboard.putData("View", m_view);
    }

    @Override
    public void run() {
        PRRConfig q = m_mech.getConfig();
        m_elevator.setLength(shoulderHeight(q));
        m_arm.setAngle(shoulderAngle(q));
        m_hand.setAngle(wristAngle(q));
    }

    ///////////////////////////////

    /**
     * config is in meters, viz wants pixels.
     * elevator zero is the floor
     */
    private static double shoulderHeight(PRRConfig q) {
        return SCALE * q.q1();
    }

    /**
     * config is in radians, viz wants degrees.
     * shoulder zero is parallel to the elevator
     */
    private static double shoulderAngle(PRRConfig q) {
        return Math.toDegrees(q.q2());
    }

    /**
     * config is in radians, viz wants degrees.
     * wrist zero is parallel to the arm.
     */
    private static double wristAngle(PRRConfig q) {
        return Math.toDegrees(q.q3());
    }
}
