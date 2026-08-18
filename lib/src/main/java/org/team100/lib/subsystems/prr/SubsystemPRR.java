package org.team100.lib.subsystems.prr;


import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsystemPRR extends Subsystem {
    /** Position, velocity, and acceleration. May compute dynamic forces too. */
    void set(PRRConfig c, PRRVelocity jv, PRRAcceleration ja);

    /** Current joint positions. */
    PRRConfig getConfig();

    /** Current joint velocities. */
    PRRVelocity getJointVelocity();
}
