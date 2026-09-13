package org.team100.lib.subsystems.rrr;

import java.util.List;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.kinematics.rrr_se2.RRRFeasibility;
import org.team100.lib.kinematics.rrr_se2.RRRKinematicsPoE;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlRRR;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.StateR1;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * This is a "helper" class for controllers that use SE2 goals.
 * 
 * Sometimes, to get between infinitesimally-adjacent poses, you have to cross a
 * boundary, e.g. a physical limit that makes a nearby configuration impossible,
 * but a different configuration (far away in joint space) possible. To travel
 * between these far-apart configurations, we use a profile.
 */
public class RRRProfile {
    private static final boolean DEBUG = false;
    private final RRRKinematicsPoE m_kinematics;
    private final RRRFeasibility m_feasibility;
    private final ProfileR1 m_profile;

    public RRRProfile(RRRKinematicsPoE kinematics, RRRFeasibility feasibility, ProfileR1 profile) {
        m_kinematics = kinematics;
        m_feasibility = feasibility;
        m_profile = profile;
    }

    /**
     * Try to go to the goal, maybe using a profile.
     * 
     * If the goal is not reachable, returns null.
     * 
     * If the goal is reachable, this chooses the closest feasible configuration,
     * and returns setpoints along a profile to get there.
     * 
     * The profile may be very unlike a straight line in SE2, e.g. to transition
     * from "elbow up" to "elbow down" means traversing the "elbow straight out"
     * configuration.
     * 
     * @param current current state (previous result)
     * @param goal    desired control
     * @param dt      provided here for testing
     */
    public ControlRRR calculate(ControlRRR current, ControlSE2 goal, double dt) {
        if (DEBUG)
            System.out.printf("RRRProfile current q [%s] qdot [%s] qddot[%s]\n",
                    current.q(), current.qdot(), current.qddot());
        Pose2d x = goal.pose();
        VelocitySE2 xdot = goal.velocity();
        AccelerationSE2 xddot = goal.acceleration();
        if (DEBUG)
            System.out.printf("RRRProfile x[%s] xdot [%s] xddot [%s]\n", StrUtil.poseStr(x), xdot, xddot);

        // Get all the possible postures.
        List<RRRConfig> qAll = m_kinematics.inverse(x, current.q().q1());
        if (qAll.isEmpty()) {
            if (DEBUG)
                System.out.println("RRRProfile: no feasible configs.");
            return null;
        }
        List<RRRConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            if (DEBUG)
                System.out.println("RRRProfile: no configs within joint and envelope limits.");
            return null;
        }
        // The nearest config for the goal
        RRRConfig goalQ = RRRConfig.nearest(qFeasible, current.q());
        // joint velocity implied by the goal control.
        RRRVelocity goalQDot = m_kinematics.inverse(goalQ, xdot);
        RRRAcceleration goalQddot = m_kinematics.inverse(goalQ, xdot, xddot);

        if (DEBUG)
            System.out.printf("RRRProfile goal q [%s] qdot [%s] qddot[%s]\n",
                    goalQ, goalQDot, goalQddot);

        double distance = current.q().euclideanDistance(goalQ);

        // is this distance "near" enough so we can use the goal as the setpoint?
        // or is it "far" enough so we need a profile in between?
        // a typical maximum single-step distance might be 0.1 rad so just use that for
        // now.
        // TODO: externalize this tolerance
        if (distance < 0.1) {
            if (DEBUG) {
                System.out.printf("RRRProfile: goal nearby: %s, %s, %s\n", goalQ, goalQDot, goalQddot);
            }
            // The goal is "near", so we can just use the goal config as the setpoint
            // and also include the goal velocity and accel.
            return new ControlRRR(goalQ, goalQDot, goalQddot);
        }
        if (DEBUG)
            System.out.println("RRRProfile: goal is far");
        // The goal is "far", so we have to use a profile to get there, and we ignore
        // the goal velocity and accel.

        // Unit vector in joint space pointing from current to goal.
        Vector<N3> m_unit = RRRConfig.unit(current.q(), goalQ);
        // The magnitude of current joint velocity in that direction. If we're
        // in the middle of the profile, then the joint velocity should be parallel
        // to the unit vector anyway.
        double projectedQdot = current.qdot().toVector().dot(m_unit);
        double projectedQddot = current.qddot().toVector().dot(m_unit);

        // We use a one-dimensional profile so that the movement is coordinated.
        // It would also be ok to use three uncoordinated profiles, I suppose.
        ControlR1 profileSetpoint = new ControlR1(0, projectedQdot, projectedQddot);
        if (DEBUG)
            System.out.printf("RRRProfile: profileSetpoint %s\n", profileSetpoint);

        // The desired joint velocity at the end.
        RRRVelocity goalQdot = m_kinematics.inverse(goalQ, xdot);
        // The magnitude of the end velocity in the direction of motion. The desired
        // end velocity may not be parallel with the direction. We could just use
        // zero here, but using the component is less wrong.
        double projectedGoalQdot = goalQdot.toVector().dot(m_unit);

        // The one-dimensional goal uses the distance metric.
        StateR1 profileGoal = new StateR1(distance, projectedGoalQdot);

        ControlR1 m_setpoint = m_profile.calculate(dt, profileSetpoint, profileGoal);
        if (DEBUG)
            System.out.printf("RRRProfile: m_setpoint %s\n", m_setpoint);

        RRRConfig q = current.q().plus(RRRConfig.fromVector(m_unit.times(m_setpoint.x())));
        RRRVelocity qdot = RRRVelocity.fromVector(m_unit.times(m_setpoint.v()));
        RRRAcceleration qddot = RRRAcceleration.fromVector(m_unit.times(m_setpoint.a()));

        ControlRRR result = new ControlRRR(q, qdot, qddot);
        if (DEBUG)
            System.out.printf("RRRProfile: result %s\n", result);
        return result;
    }

    public RRRVelocity qdot(RRRConfig q, VelocitySE2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    public RRRAcceleration qddot(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        return m_kinematics.inverse(q, xdot, xddot);
    }
}
