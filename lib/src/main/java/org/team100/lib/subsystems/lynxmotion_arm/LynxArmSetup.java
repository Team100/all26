package org.team100.lib.subsystems.lynxmotion_arm;

import org.team100.lib.hid.ControlUtil;
import org.team100.lib.kinematics.lynx_arm.AnalyticLynxArmKinematics;
import org.team100.lib.kinematics.lynx_arm.LynxArmKinematics;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Sets up the Lynxmotion arm with all axes. */
public class LynxArmSetup implements Runnable {
    private final LynxArm m_arm;
    private final LynxArmVisualizer m_viz;

    public LynxArmSetup(XboxController m_controller) {
        // for dimensions, see
        // https://wiki.lynxmotion.com/info/wiki/lynxmotion/download/ses-v1/ses-v1-robots/ses-v1-arms/al5d/WebHome/PLTW-AL5D-Guide-11.pdf
        // LynxArmKinematics kinematics = new AnalyticLynxArmKinematics(0.07, 0.146,
        // 0.187, 0.111);

        // numeric kinematics produce weird artifacts in the visualizer
        // Newton Rafston thing-a-ma-jig
        //LynxArmKinematics kinematics = new NumericLynxArmKinematics();
        // Geometry based positioning
        LynxArmKinematics kinematics = AnalyticLynxArmKinematics.real();

        m_arm = new LynxArm(kinematics);
        m_viz = new LynxArmVisualizer(m_arm::getPosition);

        // new Trigger(m_controller::getAButton).whileTrue(
        // m_arm.moveTo(new Pose3d(0.15, 0.1, 0.1, new Rotation3d(0, Math.PI / 2, 0))));
        // new Trigger(m_controller::getBButton).whileTrue(
        // m_arm.moveTo(new Pose3d(0.15, 0.1, 0, new Rotation3d(0, Math.PI / 2, 0))));
        // new Trigger(m_controller::getXButton).whileTrue(
        // m_arm.moveTo(new Pose3d(0.15, -0.1, 0.1, new Rotation3d(0, Math.PI / 2,
        // 0))));

        new Trigger(m_controller::getAButton).whileTrue(m_arm.toggleHeight());
        // open/close is left bumper
        new Trigger(m_controller::getLeftBumperButton).whileTrue(m_arm.toggleGrip());

        // this is one way to do it.
        new Trigger(m_controller::getXButton).whileTrue(
                Commands.sequence(
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, -0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, -0.15, 0.0, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, -0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, 0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, 0.15, 0.0, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.12, 0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, 0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, 0.15, 0.0, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, 0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, -0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, -0.15, 0.0, new Rotation3d(0, Math.PI / 2, 0))),
                        m_arm.moveQuicklyUntilDone(new Pose3d(0.26, -0.15, 0.05, new Rotation3d(0, Math.PI / 2, 0)))));

        // another way to do it; note this doesn't control the roll axis the same way
        new Trigger(m_controller::getYButton).whileTrue(
                Commands.sequence(
                        m_arm.up(), m_arm.openGrip(),
                        m_arm.moveXY(0.12, -0.15),
                        m_arm.down(), m_arm.closeGrip(), m_arm.up(),
                        m_arm.moveXY(0.1, -0.1),
                        m_arm.down(), m_arm.openGrip(), m_arm.up(),
                        m_arm.moveXY(0.26, 0.15),
                        m_arm.down(), m_arm.closeGrip(), m_arm.up(),
                        m_arm.moveXY(0.26, -0.15),
                        m_arm.down(), m_arm.openGrip(), m_arm.up(),
                        m_arm.moveXY(0.12, -0.15)));



        new Trigger(m_arm::getDistanceMode).onChange(     
                       Commands.sequence(
                        m_arm.up(),
                        m_arm.changeModeCmd(),
                        m_arm.down()
                ));

        
        // m_arm.setDefaultCommand(m_arm.moveHome());
        // for this to work in simulation you need to configure the sim gui keyboard
        // joystick
        // so that axes 4 (x) and 5 (y) are bound somewhere (e.g. WASD)
        m_arm.setDefaultCommand(m_arm.manual(
                () -> -0.5 * ControlUtil.deadband(m_controller.getRightY(), 0.05, 1),
                () -> -0.5 * ControlUtil.deadband(m_controller.getRightX(), 0.05, 1),
                () -> -0.5 * ControlUtil.deadband(m_controller.getLeftY(), 0.05, 1)));
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
