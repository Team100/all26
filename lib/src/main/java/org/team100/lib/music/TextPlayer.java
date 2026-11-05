package org.team100.lib.music;

/** Prints the notes being played. */
public class TextPlayer implements Player {

    private final String name;

    public TextPlayer(String name) {
        this.name = name;
    }

    @Override
    public void play(double freq) {
        System.out.printf("%s: %.0f\n", name, freq);
    }

}
