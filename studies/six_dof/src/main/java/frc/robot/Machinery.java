package frc.robot;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.six_dof.SixDofArm;
import org.team100.lib.subsystems.six_dof.SixDofVisualizer;

public class Machinery {
    private static final LoggerFactory logger = Logging.root();

    public final SixDofArm m_arm;
    public final SixDofVisualizer m_viz;

    public Machinery() {
        m_arm = new SixDofArm(logger);
        m_viz = new SixDofVisualizer(m_arm::getPose);
    }

    public void close() {
    }

    public void periodic() {
        m_viz.periodic();
    }
}
