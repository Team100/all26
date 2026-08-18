package org.team100.lib.hid;

import org.team100.frc2025.field.FieldConstants2025.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.wpilibj.GenericHID;

public class Buttons2025 {
    private final GenericHID m_controller;

    public Buttons2025(int port) {
        m_controller = new GenericHID(port);
    }

    /** Coral Scoring Levels */
    public ScoringLevel level() {
        if (l1()) {
            return ScoringLevel.L1;
        } else if (l2()) {
            return ScoringLevel.L2;
        } else if (l3()) {
            return ScoringLevel.L3;
        } else if (l4()) {
            return ScoringLevel.L4;
        }
        return ScoringLevel.NONE;
    }

    /** Coral Scoring Reef Points */
    public ReefPoint point() {
        if (a())
            return ReefPoint.A;
        if (b())
            return ReefPoint.B;
        if (c())
            return ReefPoint.C;
        if (d())
            return ReefPoint.D;
        if (e())
            return ReefPoint.E;
        if (f())
            return ReefPoint.F;
        if (g())
            return ReefPoint.G;
        if (h())
            return ReefPoint.H;
        if (i())
            return ReefPoint.I;
        if (j())
            return ReefPoint.J;
        if (k())
            return ReefPoint.K;
        if (l())
            return ReefPoint.L;
        if (ab())
            return ReefPoint.AB;
        if (cd())
            return ReefPoint.CD;
        if (ef())
            return ReefPoint.EF;
        if (gh())
            return ReefPoint.GH;
        if (ij())
            return ReefPoint.IJ;
        if (kl())
            return ReefPoint.KL;
        return ReefPoint.NONE;
    }

    /** Algae locations between the poles */
    public boolean algae() {
        return ab() || cd() || ef() || gh() || ij() || kl();
    }

    public ScoringLevel algaeLevel() {
        if (cd() || gh() || kl())
            return ScoringLevel.L2;
        if (ab() || ef() || ij())
            return ScoringLevel.L3;
        return ScoringLevel.NONE;
    }

    public boolean red1() {
        return m_controller.getRawButton(20);
    }

    public boolean red2() {
        return m_controller.getRawButton(19);
    }

    public boolean red3() {
        return m_controller.getRawButton(18);
    }

    public boolean red4() {
        return m_controller.getRawButton(17);
    }

    public boolean barge() {
        return m_controller.getRawButton(22);
    }

    public boolean l1() {
        return m_controller.getRawButton(5);
    }

    public boolean l2() {
        return m_controller.getRawButton(6);
    }

    public boolean l3() {
        return m_controller.getRawButton(7);
    }

    public boolean l4() {
        return m_controller.getRawButton(4);
    }

    public boolean a() {
        return m_controller.getRawButton(15);
    }

    public boolean b() {
        return m_controller.getRawButton(21);
    }

    public boolean c() {
        return m_controller.getRawButton(31);
    }

    public boolean d() {
        return m_controller.getRawButton(30);
    }

    public boolean e() {
        return m_controller.getRawButton(28);
    }

    public boolean f() {
        return m_controller.getRawButton(27);
    }

    public boolean g() {
        return m_controller.getRawButton(25);
    }

    public boolean h() {
        return m_controller.getRawButton(23);
    }

    public boolean i() {
        return m_controller.getRawButton(9);
    }

    public boolean j() {
        return m_controller.getRawButton(10);
    }

    public boolean k() {
        return m_controller.getRawButton(12);
    }

    public boolean l() {
        return m_controller.getRawButton(13);
    }

    public boolean ab() {
        return m_controller.getRawButton(1);
    }

    public boolean cd() {
        return m_controller.getRawButton(2);
    }

    public boolean ef() {
        return m_controller.getRawButton(29);
    }

    public boolean gh() {
        return m_controller.getRawButton(26);
    }

    public boolean ij() {
        return m_controller.getRawButton(11);
    }

    public boolean kl() {
        return m_controller.getRawButton(14);
    }

}
