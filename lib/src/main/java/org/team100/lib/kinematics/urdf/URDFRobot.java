package org.team100.lib.kinematics.urdf;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.optimization.CoordinateDescent;
import org.team100.lib.optimization.NewtonsMethod;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N6;
import org.wpilib.math.util.Nat;
import org.wpilib.math.util.Num;

/**
 * This is a partial implementation of the URDF object model.
 * 
 * The idea is to use a more general mechanism representation than the usual
 * POJO (e.g. LynxArm, which uses fields), which would allow more general
 * kinematics than the hand-made stuff we've done previously (e.g. we could try
 * Cyclic Coordinate Descent, or Gradient Descent, in a general way)
 * 
 * It would be possible to serialize this to URDF XML and vice versa, but our
 * robots are small enough that (IMHO) putting the "loader" in code would be
 * much easier than bringing in the XML overhead.
 * 
 * See http://wiki.ros.org/urdf, in particular, urdf.xsd. I transcribed directly
 * from that file. (I tried to use JAXB to auto-generate this, but gave up
 * trying to run it.)
 * 
 * This omits the collision and visual parts of the model, which are used for
 * simulation and visualization, respectively. We could add those back in if we
 * need them someday. I haven't found the URDF visualizers (e.g.
 * https://mymodelrobot.appspot.com/5629499534213120) or simulators (e.g.
 * Gazebo) to be that useful.
 * 
 * There exists SRDF, which specifies, among other things, group states, e.g.
 * home positions. I think SRDF is dead, but representing states seems fine;
 * the format is a map of joint names and values.
 * 
 * https://wiki.ros.org/srdf/review
 * 
 * @param Q the number of joints
 */
@SuppressWarnings("unused")
public class URDFRobot<Q extends Num> {

    public enum Solver {
        NEWTON, CD
    };

    private final Solver m_solver;
    private static final int DEBUG = 0;
    @SuppressWarnings("unused")
    private final String m_name;
    @SuppressWarnings("unused")
    private final List<URDFLink> m_links;
    private final List<URDFJoint> m_joints;
    private final Nat<Q> m_qDim;
    private final Vector<Q> m_dqLimit;

    public URDFRobot(
            Solver solver,
            Nat<Q> qDim,
            String name,
            List<URDFLink> links,
            List<URDFJoint> joints,
            Vector<Q> dqLimit) {
        m_solver = solver;
        m_qDim = qDim;
        m_name = name;
        m_links = links;
        m_joints = joints;
        m_dqLimit = dqLimit;
    }

    /**
     * Solve forward kinematics for all joints.
     * 
     * Key is joint name.
     * 
     * TODO: replace the map with a vector
     */
    public Map<String, Pose3d> forward(Map<String, Double> qMap) {
        Map<String, Pose3d> poses = new HashMap<>();
        for (URDFJoint joint : m_joints) {
            forward(poses, joint.name(), qMap);
        }
        return poses;
    }

    /**
     * Solve inverse kinematics for all joints using Newton's method,
     * or coordinate descent.
     * 
     * qDim indicates the dimensionality of the configuration space.
     * q0 is the initial (e.g. current) configuration.
     * 
     * the function, f, is the forward transform: it takes the joint
     * configuration vector, Q, and produces a vector representing the pose, P.
     * since the Pose is in SE(3) we have to choose a parameterization. see
     * https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
     * which has a section on SE(3) parameterizations for optimization.
     * optimizers assume euclidean spaces. they suggest optimizing on the manifold,
     * like GTSAM does.
     * 
     * ideally at each optimizer would reparameterize at each step.
     * 
     * if not, then the "error" is between two tangent vectors whose origin is far
     * away.
     * 
     * TODO: replace the returned map with a vector
     */
    public Map<String, Double> inverse(
            Vector<Q> q0,
            String jointName,
            Pose3d goal) {
        Function<Vector<Q>, Pose3d> fwd = //
                q -> forward(qMap(q)).get(jointName);

        Function<Vector<Q>, Vector<N6>> err = //
                q -> GeometryUtil.toVec(goal.minus(fwd.apply(q)));
        // q -> GeometryUtil.toVec(goal.log(fwd.apply(q)));

        // this function always uses pose3d so the goal dim is always N6.
        Nat<N6> twistDim = Nat.N6();

        // if the error is too large, then movements become jerky -- the
        // distance between current and desired pose can be within the
        // error. so keep this small, a few mm.
        double tolerance = 2e-3;

        // sometimes the solver seems to circle around the goal
        // but it only happens in the middle of movements, when the
        // initial and goal are far apart.

        // regardless, each iteration (on my fast machine) takes 50 us,
        // so we can only afford to do, say, 20, even when random-restarting, so make
        // each iteration limit low.
        int iterations = 30;

        // random restart tries to escape local minima.
        // don't try for too long, it's better to have the wrong answer sooner than the
        // right answer after a long delay.
        int restarts = 6;

        // Newton seems to oscillate.
        NewtonsMethod<Q, N6> newton = new NewtonsMethod<>(
                m_qDim, twistDim, err,
                minQ(m_qDim), maxQ(m_qDim),
                tolerance, iterations, m_dqLimit);

        // Try coordinate descent. The tolerance is quite
        // coarse, to speed it up.
        CoordinateDescent<Q> cd = new CoordinateDescent<>(
                m_qDim, q -> err.apply(q).norm(), 1e-2, 30);

        long startTime = System.nanoTime();

        Vector<Q> qVec = switch (m_solver) {
            case NEWTON -> newton.solve(q0, restarts, true);
            case CD -> cd.solve(minQ(m_qDim), q0, maxQ(m_qDim));
        };

        if (DEBUG > 0) {
            long finishTime = System.nanoTime();
            System.out.printf("ET (ms): %6.3f\n", ((double) finishTime - startTime) / 1000000);
        }
        return qMap(qVec);
    }

    URDFJoint getJoint(String name) {
        for (URDFJoint joint : m_joints) {
            if (joint.name().equals(name))
                return joint;
        }
        return null;
    }

    /**
     * Populate poses with the pose of the specified joint and those of its parent
     * chain.
     */
    private Pose3d forward(
            Map<String, Pose3d> poses,
            String jointName,
            Map<String, Double> qMap) {
        if (DEBUG > 1) {
            System.out.printf("joint %s\n", jointName);
        }
        URDFJoint joint = getJoint(jointName);
        Transform3d t = joint.transform(qMap.get(jointName));

        if (DEBUG > 1) {
            System.out.printf("transform %s\n", t);
        }
        URDFJoint parent = parentJoint(jointName);
        if (parent == null) {
            // this is the root
            Pose3d basePose = Pose3d.kZero.transformBy(t);
            poses.put(jointName, basePose);
            return basePose;
        }
        Pose3d parentPose = poses.get(parent.name());
        if (parentPose != null) {
            Pose3d newPose = parentPose.transformBy(t);
            poses.put(jointName, newPose);
            return newPose;
        }
        // if the parent hasn't been computed yet, recurse to do it.
        Pose3d newParentPose = forward(poses, parent.name(), qMap);
        Pose3d newNewPose = newParentPose.transformBy(t);
        poses.put(jointName, newNewPose);
        return newNewPose;
    }

    /** Parent joint of the specified joint. */
    private URDFJoint parentJoint(String childName) {
        URDFJoint childJoint = getJoint(childName);
        URDFLink parentLink = childJoint.parent();
        for (URDFJoint joint : m_joints) {
            if (joint.child() == parentLink)
                return joint;
        }
        return null;
    }

    /** Transform the config vector, q, into a named map. */
    public Map<String, Double> qMap(Vector<?> q) {
        Map<String, Double> qMap = new HashMap<>();
        List<URDFJoint> joints = m_joints;
        for (int i = 0; i < joints.size(); ++i) {
            URDFJoint joint = joints.get(i);
            if (joint.active()) {
                qMap.put(joint.name(), q.get(i));
            }
        }
        return qMap;
    }

    /**
     * qDim needs to match the actual number of moveable joints.
     */
    public <QQ extends Num> Vector<QQ> minQ(Nat<QQ> qDim) {
        Vector<QQ> v = new Vector<>(qDim);
        List<URDFJoint> joints = m_joints;
        for (int i = 0; i < joints.size(); ++i) {
            URDFJoint joint = joints.get(i);
            if (joint.active()) {
                v.set(i, 0, joint.limit().lower());
            }
        }
        return v;
    }

    /**
     * qDim needs to match the actual number of moveable joints.
     */
    public <QQ extends Num> Vector<QQ> maxQ(Nat<QQ> qDim) {
        Vector<QQ> v = new Vector<>(qDim);
        List<URDFJoint> joints = m_joints;
        for (int i = 0; i < joints.size(); ++i) {
            URDFJoint joint = joints.get(i);
            if (joint.active()) {
                v.set(i, 0, joint.limit().upper());
            }
        }
        return v;
    }

}
