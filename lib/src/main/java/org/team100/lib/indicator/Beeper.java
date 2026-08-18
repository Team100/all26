package org.team100.lib.indicator;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Arrays;

import org.team100.lib.music.Music;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Plays warnings using the CTRE music feature.
 * 
 * The motor music isn't particularly loud.
 * 
 * Consider using a buzzer instead.
 * 
 * @see https://mixbutton.com/music-tools/frequency-and-pitch/music-note-to-frequency-chart
 */
@SuppressWarnings("unused")
public class Beeper {
    private static final double BPM = 120;
    private static final double D5 = 587;
    private static final double Eflat5 = 622;
    private static final double F5 = 698;
    private static final double A5 = 880;
    private static final double Bflat5 = 932;
    private static final double A6 = 1670;
    private static final double Bflat6 = 1865;
    private static final double D6 = 1175;
    private static final double Eflat6 = 1245;
    private static final double F6 = 1397;
    private static final double Bflat7 = 3729;

    private final Music m_music[];

    public Beeper(Music... music) {
        m_music = music;
    }

    public Command play(double freq) {
        return parallel(Arrays.stream(m_music).map((m) -> m.play(freq)).toArray(Command[]::new));
    }

    /**
     * Three beeps and one long beep, approximately
     * 
     * | q qr q qr | q qr h |
     * 
     * at Allegro tempo.
     * 
     * Bflat5 seems not loud enough, try Bflat6.
     */
    public Command start() {
        return sequence(
                quarterNote(Bflat6),
                quarterRest(),
                quarterNote(Bflat6),
                quarterRest(),
                quarterNote(Bflat6),
                quarterRest(),
                halfNote(Bflat7),
                halfRest());
    }

    public Command progress() {
        return sequence(
                quarterNote(Bflat6),
                quarterRest());
    }

    public Command done() {
        return sequence(
                eighthNote(F6),
                eighthNote(D6),
                eighthNote(Eflat6),
                halfNote(F6));
    }

    public Command eighthNote(double freq) {
        return play(freq).withTimeout(eighth());
    }

    public Command quarterNote(double freq) {
        return play(freq).withTimeout(quarter());
    }

    public Command halfNote(double freq) {
        return play(freq).withTimeout(half());
    }

    public Command quarterRest() {
        return play(0).withTimeout(quarter());
    }

    public Command halfRest() {
        return play(0).withTimeout(half());
    }

    private double eighth() {
        return 30 / BPM;
    }

    private double quarter() {
        return 60 / BPM;
    }

    private double half() {
        return 120 / BPM;
    }

}
