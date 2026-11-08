package org.team100.lib.localization.gtsam;

import gtsam.Key;
import gtsam.Symbol;

public class Keys {
    static Key poseKey(double timestamp) {
        return poseKey((long) (timestamp * 1e6));
    }

    static Key poseKey(long t) {
        return Key.X(t);
    }

    static Key biasKey(double timestamp) {
        return biasKey((long) (timestamp * 1e6));
    }

    static Key biasKey(long t) {
        return Key.B(t);
    }

    static double sec(Symbol symbol) {
        return (double) symbol.index() / 1e6;
    }

    static long us(Symbol symbol) {
        return symbol.index();
    }

}
