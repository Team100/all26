package org.team100.lib.subsystems.rrr.commands;

import java.util.function.Supplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.hid.DriverVelocity;
import org.team100.lib.state.ControlRRR;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.rrr.RRRArm;
import org.team100.lib.subsystems.rrr.RRRProfile;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Control SE2 velocity directly, and apply using joint profiles, which means
 * sometimes switching postures.
 * 
 * NOTE! the control can be nonzero even when the stick input is exactly zero,
 * because the control might be following a profile to a "far away" goal. In
 * general, it is possible for the goal to outrun the setpoint by an arbitrary
 * amount. It might be good to run this command behind a button (e.g. left
 * trigger)
 * for that reason.
 */
public class MoveManuallySE2 extends Command {
    private static final boolean DEBUG = false;
    private static final double DT = TimedRobot100.LOOP_PERIOD_S;
    private final RRRArm m_arm;
    private final Supplier<DriverVelocity> m_input;
    private final RRRProfile m_profile;

    // for computing input acceleration
    private VelocitySE2 m_v;
    private Pose2d m_pose;

    private ControlRRR m_current;

    public MoveManuallySE2(RRRArm arm, Supplier<DriverVelocity> input, RRRProfile profile) {
        m_arm = arm;
        m_input = input;
        m_profile = profile;
        addRequirements(arm);
    }

    public Translation2d getT() {
        if (m_pose == null)
            return null;
        return m_pose.getTranslation();
    }

    @Override
    public void initialize() {
        // set current pose and config to measurement
        StateSE2 state = m_arm.getState();
        m_pose = state.pose();
        m_v = state.velocity();
        m_current = new ControlRRR(m_arm.getConfig(), m_arm.getVelocity(), new RRRAcceleration(0, 0, 0));
        if (DEBUG)
            System.out.printf("MoveManuallySE2.initialize %s, %s, %s\n", m_v, m_current.q(), m_current.qdot());
    }

    @Override
    public void execute() {
        // Input in range [-1, 1]
        DriverVelocity input = m_input.get();

        // Velocity in m/s and rad/s
        VelocitySE2 v = VelocitySE2.scale(input, 0.5, 1.5);
        if (DEBUG) {
            System.out.printf("MoveManuallySE2: v [%s] m_v [%s]\n", v, m_v);
            System.out.printf("MoveManuallySE2: current %s\n", m_current);
        }

        // when going fast at the boundary, the step size is pretty big, so we back
        // off the velocity a few times to see if we can fit a smaller step.
        for (int i = 0; i < 4; ++i) {
            // Acceleration in m/s/s and rad/s/s
            AccelerationSE2 a = v.accel(m_v, DT);
            Pose2d x = GeometryUtil.evolve(m_pose, m_v, a, DT);
            ControlSE2 goal = new ControlSE2(x, v, a);
            if (DEBUG)
                System.out.printf("MoveManuallySE2: x [%s], v [%s], a [%s]\n", x, v, a);

            ControlRRR control = m_profile.calculate(m_current, goal, DT);
            if (control != null) {
                // the goal is reachable, so update the state here, actuate, and return.
                m_current = control;
                m_v = v;
                m_pose = x;
                m_arm.set(m_current.q(), m_current.qdot(), m_current.qddot());
                if (DEBUG)
                    System.out.printf("MoveManuallySE2: reachable %d, q[%s], qdot[%s]\n",
                            i, m_current.q(), m_current.qdot());
                return;
            }
            // The goal is not reachable. Try again with less velocity.
            v = v.times(0.5);
            // Lots of accel here, but there would be anyway if you bang into the boundary.
            m_v = v;
            m_current = new ControlRRR(m_current.q(), m_current.qdot().times(0.5), new RRRAcceleration(0, 0, 0));
            if (DEBUG)
                System.out.printf("MoveManuallySE2: unreachable %d, x[%s], v[%s], q[%s], qdot[%s]\n",
                        i, StrUtil.poseStr(x), v, m_current.q(), m_current.qdot());
        }

        // We can't fit any step between the current pose and the boundary, so stop.
        m_v = VelocitySE2.ZERO;
        m_current = new ControlRRR(m_current.q(), new RRRVelocity(0, 0, 0), new RRRAcceleration(0, 0, 0));
        if (DEBUG)
            System.out.printf("MoveManuallySE2: stop q[%s], qdot[%s]\n",
                    m_current.q(), m_current.qdot());

        m_arm.set(m_current.q(), m_current.qdot(), m_current.qddot());
    }

}
