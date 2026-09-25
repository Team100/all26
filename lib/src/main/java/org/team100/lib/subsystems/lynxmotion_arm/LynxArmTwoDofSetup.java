package org.team100.lib.subsystems.lynxmotion_arm;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.subsystems.lynxmotion_arm.commands.MoveCommandTwoDof;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.geometry.Translation2d;

public class LynxArmTwoDofSetup implements Runnable {
    private final LynxArmTwoDof m_arm;
    private final LynxArmVisualizer m_viz;

    public LynxArmTwoDofSetup(Gamepad m_controller) {
        m_arm = new LynxArmTwoDof();
        m_viz = new LynxArmVisualizer(m_arm::getPose);

        MoveCommandTwoDof move1 = m_arm.moveTo(new Translation2d(0.2, 0.1));
        whileTrue(m_controller::getSouthFaceButton,
                move1);
        MoveCommandTwoDof move2 = m_arm.moveTo(new Translation2d(0.2, 0));
        whileTrue(m_controller::getEastFaceButton,
                move2);
        MoveCommandTwoDof move3 = m_arm.moveTo(new Translation2d(0.3, 0.1));
        whileTrue(m_controller::getWestFaceButton,
                move3);

        MoveCommandTwoDof move4 = m_arm.moveTo(new Translation2d(0.2, 0.05));
        MoveCommandTwoDof move5 = m_arm.moveTo(new Translation2d(0.2, 0.0));
        MoveCommandTwoDof move6 = m_arm.moveTo(new Translation2d(0.2, 0.05));
        MoveCommandTwoDof move7 = m_arm.moveTo(new Translation2d(0.4, 0.05));
        MoveCommandTwoDof move8 = m_arm.moveTo(new Translation2d(0.4, 0.0));
        MoveCommandTwoDof move9 = m_arm.moveTo(new Translation2d(0.4, 0.05));
        whileTrue(m_controller::getNorthFaceButton,
                Commands.sequence(
                        move4.until(move4::done),
                        move5.until(move5::done),
                        move6.until(move6::done),
                        move7.until(move7::done),
                        move8.until(move8::done),
                        move9.until(move9::done)));

        m_arm.setDefaultCommand(m_arm.moveHome());
    }

    @Override
    public void run() {
        m_viz.periodic();
    }
}
