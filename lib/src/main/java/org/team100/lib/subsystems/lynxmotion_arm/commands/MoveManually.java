package org.team100.lib.subsystems.lynxmotion_arm.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.subsystems.lynxmotion_arm.LynxArm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move in the XY plane, leave z alone, leave the grip alone.
 * The end-effector pose is automatic:
 * * Roll is always zero.
 * * Pitch depends on range from the origin
 * * Yaw follows the XY position.
 */
public class MoveManually extends Command {
    private static final boolean DEBUG = false;

    // prefer vertical grip. beyond this radius, extend it.
    // private static final double VERTICAL_LIMIT = 0.3;
    private static final double MIN_HEIGHT = .04004;
    private static final double MIN_RADIUS_BIRDSEYE = 0.2;
    // private static final double MIN_RADIUS = .1;
    private static final double MAX_RADIUS = 0.42;
    // meters per second
    private static final double SPEED = 0.25;
    // step per loop
    private static final double STEP = TimedRobot100.LOOP_PERIOD_S * SPEED;

    private final LynxArm m_arm;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_zSpeed;
    private double m_grip;

    private double m_x;
    private double m_y;
    private double m_z;

    // private boolean mode;
    // when mode = true, pitch = 0
    // when mode = false, pitch = 1

    public MoveManually(
            LynxArm arm,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier zSpeed) {
        m_arm = arm;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_zSpeed = zSpeed;
        addRequirements(arm);
        // mode = true;
    }

    @Override
    public void initialize() {
        m_grip = m_arm.getGrip();
        Pose3d start = m_arm.getPosition().p6();
        m_x = start.getX();
        m_y = start.getY();
        m_z = start.getZ();
    }


    @Override
    public void execute() {

        double r = Math.hypot(m_x, m_y);
        double yaw = Math.atan2(m_y, m_x);
        double r1 = Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);

        Double xNew = m_x + m_xSpeed.getAsDouble() * STEP;
        Double yNew = m_y + m_ySpeed.getAsDouble() * STEP;
        Double zNew = m_z + m_zSpeed.getAsDouble() * STEP;

        double r1New = Math.sqrt(xNew * xNew + yNew * yNew + zNew * zNew);
        double rNew = Math.hypot(xNew, yNew);
        double yawNew = Math.atan2(yNew, xNew);

        // if (DEBUG) {
        //     System.out.println("\n***EXECUTE***");
        //     System.out.printf("x %f y %f z %f yaw %f\n", m_x, m_y, m_z, yaw);
        //     System.out.printf("r %f, rNew %f", r, rNew);
        // }
        // Jittery Code:  
        // if (r1 <= MIN_RADIUS_BIRDSEYE + STEP) {
        //     // move away from the origin
        //     m_x += STEP * Math.cos(yaw);
        //     m_y += STEP * Math.sin(yaw);
        // } else if (r1 >= MAX_RADIUS - STEP) {
        //     // move away from the edge
        //     m_x -= STEP * Math.cos(yaw);
        //     m_y -= STEP * Math.sin(yaw);
        // } else if (m_x <= STEP) {
        //     // move away from the x axis
        //     m_x += STEP;
        // } else {
        //     // driver update is ok
        //     m_x = m_x + m_xSpeed.getAsDouble() * STEP;
        //     m_y = m_y + m_ySpeed.getAsDouble() * STEP;
        //     
        //     if (m_z >= MIN_HEIGHT + STEP ||  m_z + m_zSpeed.getAsDouble() * STEP > m_z){
        //         m_z = m_z + m_zSpeed.getAsDouble() * STEP;
        //     }
        // }

        //Non-Jittery Code but not great:
        // if ((r1 <= MAX_RADIUS || (Math.abs(xNew) <= Math.abs(m_x) && Math.abs(yNew) <= Math.abs(m_y))) && 
        // (r1 >= MIN_RADIUS_BIRDSEYE || (Math.abs(xNew) >= Math.abs(m_x) && Math.abs(yNew) >= Math.abs(m_y)))){
        //     m_x = xNew;
        //     m_y = yNew;
        // }
        // if (m_z >= MIN_HEIGHT || zNew > m_z){
        //     m_z = zNew;
        // }

        //Non-Jittery Code+:
        // if ((r1 <= MAX_RADIUS || r1New <= r1) && 
        // ((yaw > -1 * 1.57 || yawNew > yaw) && (yaw < 1.57 || yawNew < yaw)) &&
        // (r >= MIN_RADIUS_BIRDSEYE || rNew >= r)){
        //     m_x = xNew;
        //     m_y = yNew;
        //     if(m_z >= MIN_HEIGHT || zNew > m_z)
        //         m_z = zNew;
        // }

        // Non-Jitter with Pitch Toggle
        if ((r1 <= MAX_RADIUS || r1New <= r1) &&
        ((yaw > -1 * 1.57 || yawNew > yaw) && (yaw < 1.57 || yawNew < yaw))){
            if(r >= MIN_RADIUS_BIRDSEYE){
                m_x = xNew;
                m_y = yNew;
            }
            if(m_z >= MIN_HEIGHT || zNew > m_z)
                    m_z = zNew;
        } 

        if (DEBUG) {
            System.out.printf("r %f, rNew %f \n pitch %f \n", r, rNew, 0.0);
        }
        
        double roll = 0.0;
        // if (r < VERTICAL_LIMIT) {
        // for near targets, the grip is vertical.
        // pitch = Math.PI / 2 - 5 * m_z;
        // } else {

        //pitch = 0;
        // // further away, the pitch depends on range.
        // double s = 1 - (r - VERTICAL_LIMIT) / (MAX_RADIUS - VERTICAL_LIMIT);
        // pitch = s * Math.PI / 2;
        // }
        //m_z = Math.max(m_z, 0);

        
        
        Pose3d newPose = new Pose3d(
                new Translation3d(m_x, m_y, m_z),
                new Rotation3d(roll, 0, yaw));

        m_arm.setGrip(m_grip);
        m_arm.setPosition(newPose);




        // if(DEBUG){
        //     Pose3d end = m_arm.getPosition().p6();
        //     Double endX = end.getX();
        //     Double endY = end.getY();
        //     Double endZ = end.getZ();
        //     System.out.printf("End X %f End Y %f End Z %f", endX, endY, endZ) ;
        // }
    }

}
