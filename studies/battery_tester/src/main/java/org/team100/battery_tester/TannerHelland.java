package org.team100.battery_tester;

import org.wpilib.util.Color8Bit;

/**
 * Blackbody RGB color.
 * 
 * Ignores luminance, so it seems very wrong.
 * 
 * https://github.com/Saucistophe/libs/blob/master/global-lib/src/main/java/org/saucistophe/utils/ColorUtils.java
 * http://www.tannerhelland.com/4435/convert-temperature-rgb-algorithm-code
 */
public class TannerHelland {
    private static final boolean DEBUG = false;

    /** Blackbody radiation chromaticity only (not luminance). */
    public static Color8Bit color(double kelvin) {
        double ck = kelvin / 100;
        Color8Bit color = new Color8Bit(red(ck), green(ck), blue(ck));
        if (DEBUG)
            System.out.printf("color %s\n", color);
        return color;
    }

    /** @param ck centiKelvin */
    private static int blue(double ck) {
        if (ck >= 66)
            return 255;
        if (ck <= 19)
            return 0;
        return Math.clamp(
                (int) (138.5177312231 * Math.log(ck - 10) - 305.0447927307), 0, 255);
    }

    /** @param ck centiKelvin */
    private static int green(double ck) {
        if (ck <= 66)
            return Math.clamp(
                    (int) (99.4708025861 * Math.log(ck) - 161.1195681661), 0, 255);
        return Math.clamp(
                (int) (288.1221695283 * (Math.pow(ck - 60, -0.0755148492))), 0, 255);
    }

    /** @param ck centiKelvin */
    private static int red(double ck) {
        if (ck <= 66)
            return 255;
        return Math.clamp(
                (int) (329.698727446 * (Math.pow(ck - 60, -0.1332047592))), 0, 255);
    }
}
