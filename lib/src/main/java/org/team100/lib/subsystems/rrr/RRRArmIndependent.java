package org.team100.lib.subsystems.rrr;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
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
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.mechanism.RotaryMechanism;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.motor.ctre.Falcon500Motor;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.state.StateSE2;
import org.team100.lib.subsystems.rrr.commands.MoveManually;
import org.team100.lib.subsystems.rrr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rrr.commands.MoveWithTrajectorySE2;
import org.team100.lib.util.CanId;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.VecBuilder;


/**
 * Planar RRR arm, for training.
 */
public class RRRArmIndependent extends SubsystemBase implements RRRArm {
    private final LoggerFactory m_log;
    private final TotalCurrentLog m_currentLog;
    final RRRKinematicsPoE m_kinematics;
    final RRRDynamicsNewtonEuler m_dynamics;
    final RRRFeasibility m_feasibility;

    private final RotaryMechanism m_q1;
    private final RotaryMechanism m_q2;
    private final RotaryMechanism m_q3;

    public RRRArmIndependent(LoggerFactory parent, TotalCurrentLog currentLog) {
        m_log = parent.type(this);
        m_currentLog = currentLog;
        LoggerFactory q1 = m_log.name("q1");
        LoggerFactory q2 = m_log.name("q2");
        LoggerFactory q3 = m_log.name("q3");
        // LINK LENGTHS, METERS
        double l1 = 0.3;
        double l2 = 0.3;
        double l3 = 0.1;
        m_kinematics = new RRRKinematicsPoE(l1, l2, l3);
        m_dynamics = new RRRDynamicsNewtonEuler(
                VecBuilder.fill(0, 0, 0),
                0.1, 0.1, 0.1,
                l1, l2, l3,
                l1 / 2, l2 / 2, l3 / 2,
                0.1, 0.1, 0.1);
        m_feasibility = new RRRFeasibility(m_kinematics,
                new RRRConfig(-Math.PI / 2, -Math.PI / 2, -Math.PI / 2),
                new RRRConfig(Math.PI / 2, Math.PI / 2, Math.PI / 2));
        final Motor m1;
        final Motor m2;
        final Motor m3;
        if (Identity.instance.equals(Identity.TEST_BOARD_B0)
                || Identity.instance.equals(Identity.TEAM100_2018)) {
            m1 = new Falcon500Motor(
                    q1, m_currentLog, new CanId(5),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1));
            m2 = new Falcon500Motor(
                    q2, m_currentLog, new CanId(21),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1));
            m3 = new Neo550CANSparkMotor(
                    q3, m_currentLog, new CanId(14),
                    NeutralMode100.COAST, MotorPhase.FORWARD,
                    new CurrentLimit(20, 20), new Friction(0, 0, 0, 0),
                    PIDConstants.makePositionPID(1), 0, 0);
        } else {
            m1 = new SimulatedMotor(q1, 600);
            m2 = new SimulatedMotor(q2, 600);
            m3 = new SimulatedMotor(q3, 600);
        }
        // GEAR RATIOS
        double r1 = 7;
        double r2 = -5;
        double r3 = -12;

        m_q1 = new RotaryMechanism(q1, m1, m1.encoder(), 0, r1, -Math.PI / 2, Math.PI / 2);
        m_q2 = new RotaryMechanism(q2, m2, m2.encoder(), 0, r2, -Math.PI / 2, Math.PI / 2);
        m_q3 = new RotaryMechanism(q3, m3, m3.encoder(), 0, r3, -Math.PI / 2, Math.PI / 2);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
        m_q3.periodic();
    }

    @Override
    public double l1() {
        return m_kinematics.l1;
    }

    @Override
    public double l2() {
        return m_kinematics.l2;
    }

    @Override
    public double l3() {
        return m_kinematics.l3;
    }

    @Override
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
    @Override
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
        return RRRConfig.getBest(qFeasible, q0);
    }

    public RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public RRRAcceleration qddot(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        return m_kinematics.inverse(q, xdot, xddot);
    }

    /** Current measured configuration. */
    @Override
    public RRRConfig getConfig() {
        return new RRRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad());
    }

    /** Desired config, with limits applied. */
    public RRRConfig getConfigWithinLimits() {
        return new RRRConfig(
                m_q1.getUnwrappedPositionWithinLimits(),
                m_q2.getUnwrappedPositionWithinLimits(),
                m_q3.getUnwrappedPositionWithinLimits());
    }

    /** Current velocity. */
    public RRRVelocity getVelocity() {
        return new RRRVelocity(
                m_q1.getVelocityRad_S(),
                m_q2.getVelocityRad_S(),
                m_q3.getVelocityRad_S());
    }

    @Override
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

    @Override
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

    public Command moveManually(Gamepad controller) {
        return new MoveManually(this, controller);
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
