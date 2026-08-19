package org.team100.lib.subsystems.lynxmotion_arm.commands;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.profile.r1.WPITrapezoidProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.lynxmotion_arm.LynxArm;
import org.team100.lib.util.StrUtil;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

/**
 * Moves the arm in a straight line by interpolating the start and end
 * end-effector poses.
 * 
 * Straight lines in workspace may intersect singularities, so watch out.
 */
public class MoveCommand extends Command {
    private static final boolean DEBUG = false;
    private final LynxArm m_arm;
    private final Pose3d m_goal;
    private final ProfileR1 m_profile;

    private ControlR1 m_setpoint;
    private ModelR1 m_profileGoal;

    private Pose3d m_start;
    private double m_grip;
    private double m_distance;
    private boolean m_done;

    public MoveCommand(LynxArm arm, Pose3d goal, double velocity) {
        m_arm = arm;
        m_goal = goal;
        m_profile = new WPITrapezoidProfileR1(velocity, 1);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (DEBUG)
            System.out.println("\n***INITIALIZE***");
        m_start = m_arm.getPosition().p6();
        m_grip = m_arm.getGrip();

        // this doesn't work for twist-only moves without the minimum
        m_distance = Math.max(0.01, m_start.getTranslation().getDistance(m_goal.getTranslation()));
        m_setpoint = new ControlR1();
        m_profileGoal = new ModelR1(m_distance, 0);
        m_done = false;
        if (DEBUG) {
            System.out.printf("start %s\n", StrUtil.poseStr(m_start));
            System.out.printf("end %s\n", StrUtil.poseStr(m_goal));
        }
    }

    @Override
    public void execute() {
        if (DEBUG)
            System.out.println("\n***EXECUTE***");
        // the servo doesn't need to be commanded to maintain its position but it's a
        // good habit since some motor types do need this.
        m_arm.setGrip(m_grip);
        m_setpoint = m_profile.calculate(TimedRobot100.LOOP_PERIOD_S, m_setpoint, m_profileGoal);
        ControlR1 c = m_setpoint;
        double s = c.x() / m_distance;
        Pose3d setpoint = LynxArmPose.interpolate(m_start, m_goal, s);
        Pose3d measurement = m_arm.getPosition().p6();
        if (DEBUG) {
            System.out.printf("Goal %s Setpoint %s Measurement %s\n",
                    StrUtil.poseStr(m_goal), StrUtil.poseStr(setpoint), StrUtil.poseStr(measurement));
        }
        double togo = setpoint.getTranslation().getDistance(m_goal.getTranslation());
        Rotation3d rotTogo = setpoint.getRotation().rotateBy(m_goal.getRotation().inverse());
        double angleTogo = rotTogo.getAngle();
        Rotation3d rotTogo2 = measurement.getRotation().rotateBy(m_goal.getRotation().inverse());
        double angleTogo2 = rotTogo2.getAngle();
        if (DEBUG)
            System.out.printf("to go %f angle %f measured %f \n", togo, angleTogo,
                    angleTogo2);
        // "done" state ignores the measurement to avoid solver convergence.
        if (togo < 0.001 && Math.abs(angleTogo) < 0.001) {
            if (DEBUG)
                System.out.println("***** at goal *****");
            m_arm.setPosition(m_goal);
            m_done = true;
            return;
        }
        m_arm.setPosition(setpoint);
        if (DEBUG) {
            System.out.printf("setpoint %s\n", StrUtil.poseStr(setpoint));
            System.out.printf("goal %s\n", StrUtil.poseStr(m_goal));
            LynxArmConfig measuredConfig = m_arm.getMeasuredConfig();
            System.out.printf("measured config %s\n", measuredConfig.str());
            LynxArmConfig commandedConfig = m_arm.getInverse(setpoint);
            System.out.printf("commanded config %s\n", commandedConfig.str());
        }
    }

    public boolean done() {
        return m_done;
    }

}
