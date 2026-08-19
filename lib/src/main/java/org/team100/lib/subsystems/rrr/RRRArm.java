package org.team100.lib.subsystems.rrr;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.rrr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rrr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rrr.commands.MoveWithTrajectorySE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;

/**
 * Planar RRR arm, for training.
 */
public class RRRArm extends SubsystemBase implements PositionSubsystemSE2, PositionSubsystemRn {
    private final LoggerFactory m_log;
    final RRRKinematicsPoE m_kinematics;
    final RRRFeasibility m_feasibility;
    private final BareMotor m_q1;
    private final BareMotor m_q2;
    private final BareMotor m_q3;

    public RRRArm(LoggerFactory parent) {
        m_log = parent.type(this);
        m_kinematics = new RRRKinematicsPoE(0.3, 0.3, 0.1);
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

    /** TODO: velocity and force in config space. */
    public void setConfig(RRRConfig q) {
        m_q1.setUnwrappedPosition(q.q1(), 0, 0);
        m_q2.setUnwrappedPosition(q.q2(), 0, 0);
        m_q3.setUnwrappedPosition(q.q3(), 0, 0);
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

    /** Current configuration. */
    public RRRConfig getConfig() {
        return new RRRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad());
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

    public Pose2d pose(RRRConfig q) {
        return m_kinematics.forward(q).p4();
    }

    public void stop() {
        m_q1.stop();
        m_q2.stop();
        m_q3.stop();
    }

    // COMMANDS

    public Command warp0() {
        return run(() -> setConfig(new RRRConfig(0, 0, 0)));
    }

    public Command warp1() {
        return run(() -> setConfig(new RRRConfig(1, -1, -1)));
    }

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
    public ModelSE2 getState() {
        // TODO: add velocity
        return new ModelSE2(pose());
    }

    @Override
    public List<ModelR1> getStateRn() {
        RRRConfig q = getConfig();
        return List.of(
                new ModelR1(q.q1()),
                new ModelR1(q.q2()),
                new ModelR1(q.q3()));
    }

    @Override
    public void set(ControlSE2 setpoint) {
        // TODO: add velocity and acceleration.
        setConfig(config(setpoint.pose()));
    }

    @Override
    public void setRn(List<ControlR1> setpoint) {
        RRRConfig q = new RRRConfig(
                setpoint.get(0).x(),
                setpoint.get(1).x(),
                setpoint.get(2).x());
        setConfig(q);
    }
}
