package org.team100.lib.subsystems.prr.commands;

import java.util.function.Supplier;

import org.team100.lib.geometry.prr.PRRAcceleration;
import org.team100.lib.geometry.prr.PRRConfig;
import org.team100.lib.geometry.prr.PRRVelocity;
import org.team100.lib.hid.Velocity;
import org.team100.lib.subsystems.prr.SubsystemPRR;

import edu.wpi.first.wpilibj2.command.Command;

/** Use the operator control to "fly" the arm around in config space. */
public class ManualConfig extends Command {

    private final Supplier<Velocity> m_input;
    private final SubsystemPRR m_subsystem;

    private PRRConfig m_config;
    private PRRVelocity m_prev;

    public ManualConfig(
            Supplier<Velocity> input,
            SubsystemPRR subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_config = m_subsystem.getConfig();
        m_prev = new PRRVelocity(0, 0, 0);
    }

    @Override
    public void execute() {
        // input is [-1, 1]
        Velocity input = m_input.get();
        final double dt = 0.02;
        // control is velocity.
        // velocity in m/s and rad/s
        // we want full scale to be about 0.5 m/s and 0.5 rad/s
        PRRVelocity jv = new PRRVelocity(
                input.x() * 1.5,
                input.y() * 3,
                input.theta() * 3);
        PRRConfig newC = m_config.integrate(jv, dt);

        // impose limits; see CalgamesMech for more limits.
        if (newC.q1() < 0 || newC.q1() > 1.7) {
            newC = new PRRConfig(m_config.q1(), newC.q2(), newC.q3());
        }
        if (newC.q2() < -2 || newC.q2() > 2) {
            newC = new PRRConfig(newC.q1(), m_config.q2(), newC.q3());
        }
        if (newC.q3() < -1.5 || newC.q3() > 2.1) {
            newC = new PRRConfig(newC.q1(), newC.q2(), m_config.q3());
        }

        // recompute velocity and accel
        PRRVelocity newJv = newC.diff(m_config, dt);
        PRRAcceleration ja = newJv.diff(m_prev, dt);

        // m_subsystem.set(newC, newJv, ja);
        m_subsystem.set(newC, newJv, ja);
        m_config = newC;
        m_prev = newJv;
    }
}
