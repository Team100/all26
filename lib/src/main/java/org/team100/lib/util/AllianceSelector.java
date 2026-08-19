package org.team100.lib.util;

import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.driverstation.MatchState;

/**
 * Depending on the alliance, return the red option, or the blue option, or
 * empty if there is no alliance.
 * 
 * You could also inline this whole thing.
 */
public class AllianceSelector<T> implements Supplier<Optional<T>> {

    private final T m_red;
    private final T m_blue;

    public AllianceSelector(T red, T blue) {
        m_red = red;
        m_blue = blue;
    }

    @Override
    public Optional<T> get() {
        return MatchState.getAlliance().map(
                x -> switch (x) {
                    case RED -> m_red;
                    case BLUE -> m_blue;
                });
    }

}
