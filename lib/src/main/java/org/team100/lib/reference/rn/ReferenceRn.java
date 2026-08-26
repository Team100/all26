package org.team100.lib.reference.rn;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.StateR1;
import org.wpilib.math.util.Num;

/**
 * Provides current and next references in R^n
 */
public interface ReferenceRn<N extends Num> {
    void init();

    List<StateR1> current();

    List<ControlR1> next();

    boolean done();
}
