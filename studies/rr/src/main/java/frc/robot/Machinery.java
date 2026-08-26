package frc.robot;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.rr.RRArm;
import org.team100.lib.subsystems.rr.RRVisualizer;

public class Machinery {
    private static final LoggerFactory logger = Logging.instance().rootLogger;

    public final RRArm m_arm;
    public final RRVisualizer m_viz;

    public Machinery() {
        m_arm = new RRArm(logger);
        m_viz = new RRVisualizer(m_arm);
    }

    public void close() {
    }

    public void periodic() {
        m_viz.periodic();
    }
}
