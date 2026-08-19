package org.team100.lib.subsystems.six_dof;

import static org.wpilib.command2.Commands.run;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se3.VelocitySE3;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.kinematics.six_dof.SixDofFeasibility;
import org.team100.lib.kinematics.six_dof.SixDofKinematics;
import org.team100.lib.kinematics.six_dof.SixDofKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.six_dof.commands.MoveWithProfile;
import org.team100.lib.subsystems.six_dof.commands.MoveWithSpline;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose3d;

/**
 * Six-DOF arm, for training.
 */
public class SixDofArm extends SubsystemBase implements PositionSubsystemRn {
    private static final boolean DEBUG = false;
    private final LoggerFactory m_log;

    final SixDofKinematics m_kinematics;
    final SixDofFeasibility m_feasibility;
    private final BareMotor m_q1;
    private final BareMotor m_q2;
    private final BareMotor m_q3;
    private final BareMotor m_q4;
    private final BareMotor m_q5;
    private final BareMotor m_q6;

    public SixDofArm(LoggerFactory parent) {
        m_log = parent.type(this);

        // m_kinematics = new SixDofKinematicsAnalytic(0.1, 0.3, 0.3, 0.1);
        m_kinematics = new SixDofKinematicsPoE(0.1, 0.3, 0.3, 0.1);
        m_feasibility = new SixDofFeasibility(m_kinematics);

        m_q1 = new SimulatedBareMotor(m_log.name("q1"), 600);
        m_q2 = new SimulatedBareMotor(m_log.name("q2"), 600);
        m_q3 = new SimulatedBareMotor(m_log.name("q3"), 600);
        m_q4 = new SimulatedBareMotor(m_log.name("q4"), 600);
        m_q5 = new SimulatedBareMotor(m_log.name("q5"), 600);
        m_q6 = new SimulatedBareMotor(m_log.name("q6"), 600);
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
        List<SixDofConfig> qAll = m_kinematics.inverse(p, q0.q1(), q0.q4());
        List<SixDofConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.poseStr(p));
            return null;
        }
        return getBest(qFeasible, q0);
    }

    public void setPosition(Pose3d p) {
        SixDofConfig q = config(p);
        if (q == null)
            return;
        setConfig(q);
    }

    /** TODO: velocity and force in config space. */
    public void setConfig(SixDofConfig q) {
        m_q1.setUnwrappedPosition(q.q1(), 0, 0);
        m_q2.setUnwrappedPosition(q.q2(), 0, 0);
        m_q3.setUnwrappedPosition(q.q3(), 0, 0);
        m_q4.setUnwrappedPosition(q.q4(), 0, 0);
        m_q5.setUnwrappedPosition(q.q5(), 0, 0);
        m_q6.setUnwrappedPosition(q.q6(), 0, 0);
    }

    /**
     * Choose config "closest" to q0, using the (non-Euclidean) config distance
     * metric.
     */
    SixDofConfig getBest(List<SixDofConfig> qAll, SixDofConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        SixDofConfig best = qAll.get(0);
        for (SixDofConfig q : qAll) {
            double d = q0.distance(q);
            if (DEBUG)
                System.out.printf("q0 %s q %s distance %6.3f\n", q0, q, d);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
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

    private SixDofPose pose(SixDofConfig q) {
        return m_kinematics.forward(q);
    }

    public Command warp0() {
        return run(() -> setConfig(new SixDofConfig(0, 0, 0, 0, 0, 0)));
    }

    public Command warp1() {
        return run(() -> setConfig(new SixDofConfig(0, 1, -1, 0, -1, 0)));
    }

    public MoveAndHold move0() {
        return new MoveWithProfile(this, pose(new SixDofConfig(0, 0, 0, 0, 0, 0)).p7());
    }

    public MoveAndHold move1() {
        return new MoveWithProfile(this, pose(new SixDofConfig(0, 1, -1, 0, -1, 0)).p7());
    }

    public MoveAndHold move(Pose3d goal) {
        return new MoveWithProfile(this, goal);
    }

    public MoveAndHold moveSplined(VelocitySE3 x0dot, Pose3d x1, VelocitySE3 x1dot) {
        return new MoveWithSpline(m_log, this, x0dot, x1, x1dot);
    }

    @Override
    public void setRn(List<ControlR1> setpoint) {
        SixDofConfig q = new SixDofConfig(
                setpoint.get(0).x(),
                setpoint.get(1).x(),
                setpoint.get(2).x(),
                setpoint.get(3).x(),
                setpoint.get(4).x(),
                setpoint.get(5).x());
        setConfig(q);
    }

    @Override
    public List<ModelR1> getStateRn() {
        SixDofConfig q = getConfig();
        return List.of(
                new ModelR1(q.q1()),
                new ModelR1(q.q2()),
                new ModelR1(q.q3()),
                new ModelR1(q.q4()),
                new ModelR1(q.q5()),
                new ModelR1(q.q6()));
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
