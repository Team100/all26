package org.team100.frc2025.CalgamesArm;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.kinematics.prr.PRRKinematics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.path.se2.PathSE2Factory;
import org.team100.lib.subsystems.se2.commands.GoToPosePosition;
import org.team100.lib.trajectory.se2.TrajectorySE2Factory;
import org.team100.lib.trajectory.se2.TrajectorySE2Planner;
import org.team100.lib.trajectory.se2.constraint.ConstantConstraint;
import org.team100.lib.trajectory.se2.constraint.TimingConstraint;
import org.team100.lib.trajectory.se2.constraint.TorqueConstraint;
import org.team100.lib.trajectory.se2.constraint.YawRateConstraint;

import edu.wpi.first.wpilibj2.command.Command;

/** Make a trajectory from the start to the end and follow it. */
public class MechTrajectories extends Command {

    private final LoggerFactory m_log;
    private final CalgamesMech m_subsystem;
    private final TrajectorySE2Planner m_planner;

    public MechTrajectories(
            LoggerFactory parent,
            CalgamesMech mech,
            PRRKinematics k) {
        m_log = parent.type(this);
        m_subsystem = mech;
        List<TimingConstraint> c = new ArrayList<>();

        // These are known to work, but suboptimal.
        c.add(new ConstantConstraint(10, 5));
        c.add(new YawRateConstraint(10, 5));
        // This is new
        c.add(new TorqueConstraint(20));

        // ALERT!
        // The parameters here used to be double these values;
        // These finer grains make smoother paths and schedules but
        // take longer to compute, so if it takes too long, make these
        // numbers bigger!
        TrajectorySE2Factory trajectoryFactory = new TrajectorySE2Factory(c);
        PathSE2Factory pathFactory = new PathSE2Factory(0.05, 0.01, 0.1);
        m_planner = new TrajectorySE2Planner(pathFactory, trajectoryFactory);
    }

    /** A command that goes from the start to the end and then finishes. */
    public Command terminal(String name, DirectionSE2 startCourse, WaypointSE2 end) {
        MoveAndHold f = new GoToPosePosition(
                m_log, m_subsystem, startCourse, end, m_planner);
        return f.until(f::isDone).withName(name);
    }

    /** A command that goes from the start to the end and then waits forever. */
    public MoveAndHold endless(String name, DirectionSE2 startCourse, WaypointSE2 end) {
        GoToPosePosition c = new GoToPosePosition(
                m_log, m_subsystem, startCourse, end, m_planner);
        c.setName(name);
        return c;
    }

}
