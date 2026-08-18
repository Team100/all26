package org.team100.lib.subsystems.five_bar;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The pen is mounted on the end of the linkage, so it could be considered part
 * of the linkage subsystem, but in fact the pen is completely independent, so
 * it's a separate subsystem.
 * 
 * Alternate arrangements are possible, for example the linkage could be
 * controlled via a Pose3d, in which case the pen would respond to the "z" part.
 * But in this case the pen really only has two useful states, so the Pose3d is
 * kind of overkill.
 */
public class Pen extends SubsystemBase {
    // These are fake values
    private static final double UP = 0.25;
    private static final double DOWN = 0.75;

    private final Servo m_servo;

    public Pen() {
        m_servo = new Servo(1);
    }

    private void set(double x) {
        m_servo.set(x);
    }

    public Command up() {
        return runOnce(() -> set(UP));
    }

    public Command down() {
        return runOnce(() -> set(DOWN));
    }
}
