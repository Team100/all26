package org.team100.lib.visualization;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Visualization for anything that moves, if the position isn't important, e.g.
 * shooter wheels. The position shown is not calibrated in any way, it's just
 * meant to show "it's spinning" or "it's not spinning".
 */
public class SpinnyVisualization implements Runnable {
    private final DoubleSupplier m_source;
    private final double m_scale;
    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_arm;
    private double angle;

    /**
     * @param source of velocity, unit doesn't matter
     * @param name   for the widget
     * @param scale  multiply times the observation
     */
    public SpinnyVisualization(DoubleSupplier source, String name, double scale) {
        m_source = source;
        m_scale = scale;
        m_view = new Mechanism2d(100, 100);
        m_root = m_view.getRoot("root", 50, 50);
        m_arm = new MechanismLigament2d("arm", 40, 0);
        m_root.append(m_arm);
        SmartDashboard.putData(name, m_view);
    }

    @Override
    public void run() {
        angle += m_scale * m_source.getAsDouble();
        if (Logging.instance().getLevel().admit(Level.TRACE)) {
            m_arm.setAngle(angle);
        }
    }

}
