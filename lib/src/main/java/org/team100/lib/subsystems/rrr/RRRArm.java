package org.team100.lib.subsystems.rrr;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.dynamics.rrr.RRRDynamicsNewtonEuler;
import org.team100.lib.dynamics.rrr.RRREffort;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.rrr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rrr.commands.MoveWithTrajectorySE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N3;

/**
 * Planar RRR arm, for training.
 */
public class RRRArm extends SubsystemBase implements PositionSubsystemSE2, PositionSubsystemRn<N3> {
    private final LoggerFactory m_log;
    final RRRKinematicsPoE m_kinematics;
    final RRRDynamicsNewtonEuler m_dynamics;
    final RRRFeasibility m_feasibility;
    private final BareMotor m_q1;
    private final BareMotor m_q2;
    private final BareMotor m_q3;

    public RRRArm(LoggerFactory parent) {
        m_log = parent.type(this);
        m_kinematics = new RRRKinematicsPoE(0.3, 0.3, 0.1);
        m_dynamics = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(0, 0, 0), 0.1, 0.1, 0.1, 0.3, 0.3, 0.1, 0.15, 0.15, 0.05, 0.1, 0.1, 0.1);
        m_feasibility = new RRRFeasibility(m_kinematics);
        m_q1 = new SimulatedBareMotor(m_log.name("q1"), 600);
        m_q2 = new SimulatedBareMotor(m_log.name("q2"), 600);
        m_q3 = new SimulatedBareMotor(m_log.name("q3"), 600);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
        m_q3.periodic();
    }

    public void set(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot) {
        RRREffort f = m_dynamics.effort(q, qdot, qddot);
        set(q, qdot, f);
    }

    public void set(RRRConfig q, RRRVelocity qdot, RRREffort f) {
        m_q1.setUnwrappedPosition(q.q1(), qdot.q1dot(), f.t1());
        m_q2.setUnwrappedPosition(q.q2(), qdot.q2dot(), f.t2());
        m_q3.setUnwrappedPosition(q.q3(), qdot.q3dot(), f.t3());
    }

    /**
     * Choose the feasible config closest to the current config.
     * 
     * @param p tool center point pose
     */
    public RRRConfig config(Pose2d p) {
        RRRConfig q0 = getConfig();
        List<RRRConfig> qAll = m_kinematics.inverse(p, q0.q1());
        if (qAll.isEmpty()) {
            System.out.println("no solution for pose " + StrUtil.poseStr(p));
            return null;
        }
        List<RRRConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.poseStr(p));
            return null;
        }
        return getBest(qFeasible, q0);
    }

    public RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public RRRAcceleration qddot(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        return m_kinematics.inverse(q, xdot, xddot);
    }

    /** Current configuration. */
    public RRRConfig getConfig() {
        return new RRRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad());
    }

    /** Current velocity. */
    public RRRVelocity getVelocity() {
        return new RRRVelocity(
                m_q1.getVelocityRad_S(),
                m_q2.getVelocityRad_S(),
                m_q3.getVelocityRad_S());
    }

    /**
     * Choose config "closest" to q0, using the (non-Euclidean) config distance
     * metric.
     */
    RRRConfig getBest(List<RRRConfig> qAll, RRRConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        RRRConfig best = qAll.get(0);
        for (RRRConfig q : qAll) {
            double d = q0.distance(q);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
    }

    public Pose2d pose() {
        return pose(getConfig());
    }

    public VelocitySE2 velocity() {
        return velocity(getConfig(), getVelocity());
    }

    public Pose2d pose(RRRConfig q) {
        return m_kinematics.forward(q).p4();
    }

    public VelocitySE2 velocity(RRRConfig q, RRRVelocity qdot) {
        return m_kinematics.forward(q, qdot);
    }

    public void stop() {
        m_q1.stop();
        m_q2.stop();
        m_q3.stop();
    }

    // COMMANDS

    public MoveAndHold moveProfiled(ProfileR1 profile, Pose2d goal) {
        return new MoveWithProfile(this, profile, goal);
    }

    public MoveAndHold moveTrajSE2(Pose2d goal, double speed) {
        return new MoveWithTrajectorySE2(m_log, this, goal, speed);
    }

    public MoveAndHold moveSplined(VelocitySE2 x0dot, Pose2d x1, VelocitySE2 x1dot) {
        return new MoveWithSpline(m_log, this, x0dot, x1, x1dot);
    }

    @Override
    public StateSE2 getState() {
        return new StateSE2(pose(), velocity());
    }

    @Override
    public List<StateR1> getStateRn() {
        RRRConfig q = getConfig();
        RRRVelocity qdot = getVelocity();
        return List.of(
                new StateR1(q.q1(), qdot.q1dot()),
                new StateR1(q.q2(), qdot.q2dot()),
                new StateR1(q.q3(), qdot.q3dot()));
    }

    @Override
    public void set(ControlSE2 setpoint) {
        Pose2d x = setpoint.pose();
        VelocitySE2 xdot = setpoint.velocity();
        AccelerationSE2 xddot = setpoint.acceleration();
        RRRConfig q = config(x);
        RRRVelocity qdot = qdot(q, xdot);
        RRRAcceleration qddot = qddot(q, xdot, xddot);
        set(q, qdot, qddot);
    }

    @Override
    public void setRn(List<ControlR1> p) {
        ControlR1 c1 = p.get(0);
        ControlR1 c2 = p.get(1);
        ControlR1 c3 = p.get(2);
        RRRConfig q = new RRRConfig(c1.x(), c2.x(), c3.x());
        RRRVelocity qdot = new RRRVelocity(c1.v(), c2.v(), c3.v());
        RRRAcceleration qddot = new RRRAcceleration(c1.a(), c2.a(), c3.a());
        set(q, qdot, qddot);
    }
}
