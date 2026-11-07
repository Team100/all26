package org.team100.lib.localization.gtsam;

import gtsam.Key;
import gtsam.Symbol;

public class Keys {
    static Key poseKey(double timestamp) {
        return Key.X((long) (timestamp * 1e6));
    }

    static Key biasKey(double timestamp) {
        return Key.B((long) (timestamp * 1e6));
    }

    static double timestamp(Symbol symbol) {
        return (double) symbol.index() / 1e6;
    }

}
