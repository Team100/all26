package org.team100.lib.subsystems.rr;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.dynamics.rr.RRDynamics;
import org.team100.lib.dynamics.rr.RRDynamicsAnalytic;
import org.team100.lib.dynamics.rr.RREffort;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.kinematics.rr.RRFeasibility;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlR2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateR2;
import org.team100.lib.subsystems.r2.PositionSubsystemR2;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.rr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rr.commands.MoveWithTrajectoryR2;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.numbers.N2;

/**
 * Planar RR arm, for training.
 */
public class RRArm extends SubsystemBase
        implements PositionSubsystemR2, PositionSubsystemRn<N2> {
    private final LoggerFactory m_log;
    final RRKinematics m_kinematics;
    final RRDynamics m_dynamics;
    final RRFeasibility m_feasibility;
    private final Motor m_q1;
    private final Motor m_q2;

    public RRArm(LoggerFactory parent) {
        m_log = parent.type(this);
        m_kinematics = new RRKinematics(0.3, 0.3);
        m_dynamics = new RRDynamicsAnalytic(
                0.1, 0.1, 0.3, 0.3, 0.15, 0.15, 0.1, 0.1);
        m_feasibility = new RRFeasibility(
                m_kinematics,
                new RRConfig(-Math.PI / 2, -3),
                new RRConfig(Math.PI / 2, 3));
        m_q1 = new SimulatedMotor(m_log.name("q1"), 600);
        m_q2 = new SimulatedMotor(m_log.name("q2"), 600);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
    }

    public void set(RRConfig q, RRVelocity qdot, RRAcceleration qddot) {
        RREffort f = m_dynamics.effort(q, qdot, qddot);
        set(q, qdot, f);
    }

    public void set(RRConfig q, RRVelocity qdot, RREffort f) {
        m_q1.setUnwrappedPosition(q.q1(), qdot.q1dot(), f.t1());
        m_q2.setUnwrappedPosition(q.q2(), qdot.q2dot(), f.t2());
    }

    /**
     * Choose the feasible config closest to the current config.
     * 
     * @param p tool center point translation
     */
    public RRConfig config(Translation2d p) {
        RRConfig q0 = getConfig();
        List<RRConfig> qAll = m_kinematics.inverse(p, q0.q1());
        if (qAll.isEmpty()) {
            System.out.println("no solution for pose " + StrUtil.transStr(p));
            return null;
        }
        List<RRConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.transStr(p));
            return null;
        }
        return RRConfig.getBest(qFeasible, q0);
    }

    public RRVelocity qdot(RRConfig q, VelocityR2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public RRAcceleration qddot(RRConfig q, VelocityR2 xdot, AccelerationR2 xddot) {
        return m_kinematics.inverse(q, xdot, xddot);
    }

    /** Current configuration. */
    public RRConfig getConfig() {
        return new RRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad());
    }

    /** Current velocity. */
    public RRVelocity getVelocity() {
        return new RRVelocity(
                m_q1.getVelocityRad_S(),
                m_q2.getVelocityRad_S());
    }

    public Translation2d translation() {
        return translation(getConfig());
    }

    public VelocityR2 velocity() {
        return velocity(getConfig(), getVelocity());
    }

    public Translation2d translation(RRConfig q) {
        return m_kinematics.forward(q).p2();
    }

    public VelocityR2 velocity(RRConfig q, RRVelocity qdot) {
        return m_kinematics.forward(q, qdot);
    }

    public void stop() {
        m_q1.stop();
        m_q2.stop();
    }

    // COMMANDS

    public MoveAndHold moveProfiled(ProfileR1 profile, Translation2d goal) {
        return new MoveWithProfile(this, profile, goal);
    }

    public MoveAndHold moveTrajSE2(Pose2d goal, double speed) {
        return new MoveWithTrajectoryR2(m_log, this, goal, speed);
    }

    public MoveAndHold moveSplined(VelocityR2 x0dot, Translation2d x1, VelocityR2 x1dot) {
        return new MoveWithSpline(m_log, this, x0dot, x1, x1dot);
    }

    @Override
    public StateR2 getState() {
        return new StateR2(translation(), velocity());
    }

    @Override
    public List<StateR1> getStateRn() {
        RRConfig q = getConfig();
        RRVelocity qdot = getVelocity();
        return List.of(
                new StateR1(q.q1(), qdot.q1dot()),
                new StateR1(q.q2(), qdot.q2dot()));
    }

    /** Ignores rotation */
    @Override
    public void set(ControlR2 setpoint) {
        Translation2d x = setpoint.translation();
        VelocityR2 xdot = setpoint.velocity();
        AccelerationR2 xddot = setpoint.acceleration();
        RRConfig q = config(x);
        RRVelocity qdot = qdot(q, xdot);
        RRAcceleration qddot = qddot(q, xdot, xddot);
        set(q, qdot, qddot);
    }

    @Override
    public void setRn(List<ControlR1> p) {
        ControlR1 c1 = p.get(0);
        ControlR1 c2 = p.get(1);
        RRConfig q = new RRConfig(c1.x(), c2.x());
        RRVelocity qdot = new RRVelocity(c1.v(), c2.v());
        RRAcceleration qddot = new RRAcceleration(c1.a(), c2.a());
        set(q, qdot, qddot);
    }
}
