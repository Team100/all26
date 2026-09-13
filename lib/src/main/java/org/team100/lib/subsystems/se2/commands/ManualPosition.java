package org.team100.lib.subsystems.se2.commands;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverVelocity;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;



/**
 * Use the operator control to "fly" a positional planar subsystem around in
 * cartesian space.
 */
public class ManualPosition extends Command {
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    // scale stick input [-1,1] to movement in m/s and rad/s
    private static final double SCALE = 0.25;

    private final Supplier<DriverVelocity> m_input;
    private final PositionSubsystemSE2 m_subsystem;

    // for computing acceleration
    private VelocitySE2 m_v;
    private Pose2d m_pose;

    public ManualPosition(
            Supplier<DriverVelocity> input,
            PositionSubsystemSE2 subsystem) {
        m_input = input;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_pose = m_subsystem.getState().pose();
        m_v = VelocitySE2.ZERO;
    }

    @Override
    public void execute() {
        // Input in range [-1, 1]
        DriverVelocity input = m_input.get();
        // Velocity in m/s and rad/s
        VelocitySE2 v = VelocitySE2.scale(input, SCALE, SCALE);
        // Acceleration in m/s/s and rad/s/s
        AccelerationSE2 a = v.accel(m_v, DT);
        // Find new position using *previous* velocity and accel
        // x1 = x0 + v0 dt + 1/2 a dt^2
        m_pose = GeometryUtil.evolve(m_pose, m_v, a, DT);
        m_subsystem.set(new ControlSE2(m_pose, v, a));
    }
}
