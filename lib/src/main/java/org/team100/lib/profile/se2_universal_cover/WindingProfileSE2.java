package org.team100.lib.profile.se2_universal_cover;

import org.team100.lib.state.WindingControlSE2;
import org.team100.lib.state.WindingStateSE2;

public interface WindingProfileSE2 {

    /**
     * For coordinated profiles, do whatever precalculation is required. *
     * 
     * @param i initial
     * @param g goal
     */
    void solve(WindingStateSE2 i, WindingStateSE2 g);

    /**
     * Compute the control for the end of the next time step.
     * 
     * @param i initial
     * @param g goal
     * @return control
     */
    WindingControlSE2 calculate(WindingStateSE2 i, WindingStateSE2 g);

}