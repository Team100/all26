package org.team100.lib.visualization;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Visualization for any single rotary thing. */
public class ArmVisualization implements Runnable {
    private final DoubleSupplier m_source;
    private final double m_offsetRad;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_arm;

    /**
     * @param source    of angle in radians
     * @param name      for the widget
     * @param offsetRad add to the supplied angle; the widget zero is horizontal
     */
    public ArmVisualization(DoubleSupplier source, String name, double offsetRad) {
        m_source = source;
        m_offsetRad = offsetRad;
        m_view = new Mechanism2d(100, 100);
        m_root = m_view.getRoot("root", 50, 50);
        m_arm = new MechanismLigament2d("arm", 40, 0);
        m_root.append(m_arm);
        SmartDashboard.putData(name, m_view);
    }

    @Override
    public void run() {
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            m_arm.setAngle(Math.toDegrees(m_source.getAsDouble() + m_offsetRad));
        }
    }
}
