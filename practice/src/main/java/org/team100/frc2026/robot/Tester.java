package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.util.Banner;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Tester {

    private static final String TEST = "TEST: ";
    private final Machinery m_machinery;
    private final Alert m_alert;

    public Tester(Machinery machinery) {
        m_machinery = machinery;
        m_alert = new Alert(text("test"), AlertType.kInfo);
        m_alert.set(false);
    }

    /**
     * TEST ALL MOVEMENTS
     * 
     * For pre- and post-match testing.
     * 
     * Enable "test" mode and press driver "a" and "b" together.
     * (in simulation this is buttons 1 and 2, or "z" and "x" on the keyboard)
     * 
     * =================== DANGER DANGER DANGER ===================
     *
     * THIS WILL MOVE THE ROBOT VERY FAST!
     *
     * DO NOT RUN WITH WHEELS ON THE FLOOR!
     *
     * =================== DANGER DANGER DANGER ===================
     */
    public Command prematch() {
        return sequence(
                start(),
                announce("AHEAD SLOW"),
                m_machinery.m_drive.aheadSlow().withTimeout(1),
                m_machinery.m_drive.stopOnce(),
                announce("RIGHTWARD SLOW"),
                m_machinery.m_drive.rightwardSlow().withTimeout(1),
                m_machinery.m_drive.stopOnce(),
                announce("SPIN LEFT"),
                m_machinery.m_drive.spinLeft().withTimeout(1),
                m_machinery.m_drive.stopOnce(),
                announce("INTAKE EXTEND OUT"),
                m_machinery.m_intakeExtend.goToExtendedPosition().withTimeout(1),
                m_machinery.m_intakeExtend.stopOnce(),
                announce("INTAKE RUN"),
                m_machinery.m_intake.intake().withTimeout(1),
                m_machinery.m_intake.stopOnce(),
                announce("INTAKE EXTEND IN"),
                m_machinery.m_intakeExtend.goToRetractedPosition().withTimeout(1),
                m_machinery.m_intakeExtend.stopOnce(),
                announce("SHOOTER RUN"),
                m_machinery.m_shooter.testRun().withTimeout(1),
                m_machinery.m_shooter.stopOnce(),
                done()).finallyDo(() -> m_alert.set(false))
                .withName("Test all movements");
    }

    public Command prompt() {
        return show("push a and b together to start");
    }

    public Command start() {
        return sequence(
                show("*** WARNING! MOTION STARTS IN 4 SECONDS ***"),
                Commands.runOnce(Banner::printCaution),
                m_machinery.m_beeper.start());
    }

    public Command announce(String str) {
        return sequence(
                show(str),
                m_machinery.m_beeper.progress());
    }

    public Command done() {
        return sequence(
                show("TESTING FINISHED!"),
                m_machinery.m_beeper.done());
    }

    /** Show the text as an alert and on the console. */
    private Command show(String text) {
        return parallel(
                runOnce(() -> {
                    m_alert.setText(text(text));
                    m_alert.set(true);
                }),
                print(text(text)));
    }

    private String text(String text) {
        return TEST + text;
    }

}
