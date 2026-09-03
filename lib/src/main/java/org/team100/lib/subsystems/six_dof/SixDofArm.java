package org.team100.lib.subsystems.six_dof;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.dynamics.six_dof.SixDofDynamicsNewtonEuler;
import org.team100.lib.dynamics.six_dof.SixDofEffort;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.kinematics.six_dof.SixDofFeasibility;
import org.team100.lib.kinematics.six_dof.SixDofKinematics;
import org.team100.lib.kinematics.six_dof.SixDofKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.Motor;
import org.team100.lib.motor.sim.SimulatedMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.six_dof.commands.MoveWithProfile;
import org.team100.lib.subsystems.six_dof.commands.MoveWithSpline;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.numbers.N6;

/**
 * Six-DOF arm, for training.
 */
public class SixDofArm extends SubsystemBase implements PositionSubsystemRn<N6> {
    private final LoggerFactory m_log;

    final SixDofKinematics m_kinematics;
    final SixDofDynamicsNewtonEuler m_dynamics;
    final SixDofFeasibility m_feasibility;
    private final Motor m_q1;
    private final Motor m_q2;
    private final Motor m_q3;
    private final Motor m_q4;
    private final Motor m_q5;
    private final Motor m_q6;

    public SixDofArm(LoggerFactory parent) {
        m_log = parent.type(this);

        // m_kinematics = new SixDofKinematicsAnalytic(0.1, 0.3, 0.3, 0.1);
        m_kinematics = new SixDofKinematicsPoE(0.1, 0.3, 0.3, 0.1);
        m_dynamics = new SixDofDynamicsNewtonEuler(
                0.1, 0.3, 0.3, 0.1,
                0.5, 1, 1, 0.5);
        m_feasibility = new SixDofFeasibility(m_kinematics);

        m_q1 = new SimulatedMotor(m_log.name("q1"), 600);
        m_q2 = new SimulatedMotor(m_log.name("q2"), 600);
        m_q3 = new SimulatedMotor(m_log.name("q3"), 600);
        m_q4 = new SimulatedMotor(m_log.name("q4"), 600);
        m_q5 = new SimulatedMotor(m_log.name("q5"), 600);
        m_q6 = new SimulatedMotor(m_log.name("q6"), 600);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
        m_q3.periodic();
        m_q4.periodic();
        m_q5.periodic();
        m_q6.periodic();
    }

    /**
     * @param p tool center point pose, aimed at +z
     */
    public SixDofConfig config(Pose3d p) {
        SixDofConfig q0 = getConfig();
        List<SixDofConfig> qAll = m_kinematics.inverse(p, q0.q1(), q0.q2(), q0.q4());
        List<SixDofConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.poseStr(p));
            return null;
        }
        return SixDofConfig.getBest(qFeasible, q0);
    }

    public void set(SixDofConfig q, SixDofVelocity qdot, SixDofAcceleration qddot) {
        SixDofEffort f = m_dynamics.effort(q, qdot, qddot);
        set(q, qdot, f);
    }

    public void set(SixDofConfig q, SixDofVelocity qdot, SixDofEffort f) {
        m_q1.setUnwrappedPosition(q.q1(), qdot.q1dot(), f.t1());
        m_q2.setUnwrappedPosition(q.q2(), qdot.q2dot(), f.t2());
        m_q3.setUnwrappedPosition(q.q3(), qdot.q3dot(), f.t3());
        m_q4.setUnwrappedPosition(q.q4(), qdot.q4dot(), f.t4());
        m_q5.setUnwrappedPosition(q.q5(), qdot.q5dot(), f.t5());
        m_q6.setUnwrappedPosition(q.q6(), qdot.q6dot(), f.t6());
    }

    public SixDofVelocity qdot(SixDofConfig q, VelocitySE3 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public SixDofConfig getConfig() {
        return new SixDofConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad(),
                m_q4.getUnwrappedPositionRad(),
                m_q5.getUnwrappedPositionRad(),
                m_q6.getUnwrappedPositionRad());
    }

    public SixDofPose getPose() {
        return pose(getConfig());
    }

    public SixDofPose pose(SixDofConfig q) {
        return m_kinematics.forward(q);
    }

    // COMMANDS

    public Command warp0() {
        return run(() -> set(
                SixDofConfig.zero(),
                SixDofVelocity.zero(),
                SixDofAcceleration.zero()));
    }

    public Command warp1() {
        return run(() -> set(
                new SixDofConfig(0, 1, -1, 0, -1, 0),
                SixDofVelocity.zero(),
                SixDofAcceleration.zero()));
    }

    public MoveAndHold move0(ProfileR1 profile) {
        return new MoveWithProfile(
                this, profile, pose(new SixDofConfig(0, 0, 0, 0, 0, 0)).p7());
    }

    public MoveAndHold move1(ProfileR1 profile) {
        return new MoveWithProfile(
                this, profile, pose(new SixDofConfig(0, 1, -1, 0, -1, 0)).p7());
    }

    public MoveWithProfile move(ProfileR1 profile, Pose3d goal) {
        return new MoveWithProfile(this, profile, goal);
    }

    public MoveAndHold moveSplined(VelocitySE3 x0dot, Pose3d x1, VelocitySE3 x1dot) {
        return new MoveWithSpline(m_log, this, x0dot, x1, x1dot);
    }

    @Override
    public void setRn(List<ControlR1> setpoint) {
        SixDofConfig q = SixDofConfig.fromList(setpoint);
        SixDofVelocity qdot = SixDofVelocity.fromList(setpoint);
        SixDofAcceleration qddot = SixDofAcceleration.fromList(setpoint);
        set(q, qdot, qddot);
    }

    @Override
    public List<StateR1> getStateRn() {
        SixDofConfig q = getConfig();
        return List.of(
                new StateR1(q.q1()),
                new StateR1(q.q2()),
                new StateR1(q.q3()),
                new StateR1(q.q4()),
                new StateR1(q.q5()),
                new StateR1(q.q6()));
    }

    @Override
    public void stop() {
        m_q1.stop();
        m_q2.stop();
        m_q3.stop();
        m_q4.stop();
        m_q5.stop();
        m_q6.stop();
    }

}
