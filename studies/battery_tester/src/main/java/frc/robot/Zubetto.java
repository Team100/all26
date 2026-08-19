package frc.robot;

import org.wpilib.util.Color8Bit;

/**
 * Blackbody chromaticity and luminance, with some hacky luminance scaling to
 * produce a color that "looks good."
 * 
 * https://github.com/zubetto/BlackBodyRadiation/blob/main/BlackBodyRadiation.hlsl
 */
public class Zubetto {
    private static final boolean DEBUG = false;

    /** Blackbody color including hacky luminance. */
    public static Color8Bit color(double T) {
        if (T <= 0.0)
            return new Color8Bit(0, 0, 0);

        // --- Effective radiance in W/(sr*m2) ---

        double a = 230141698.067 / (Math.exp(25724.2 / T) - 1.0);

        // luminance Lv = Km*ChromaRadiance.a in cd/m2, where Km = 683.002 lm/W
        // (resulting in the perceived luminance, lm/(sr*m2))

        // --- Chromaticity in linear sRGB ---
        // (i.e. color luminance Y = dot({r,g,b}, {0.2126, 0.7152, 0.0722}) = 1)

        double r = r(T);
        double g = g(T);
        double b = b(T);

        // The actual range of luminance is very high, not useful for the
        // visualizer, so, use a nonlinear scaling.
        // Subjective "brightness" data:
        // https://psychology.stackexchange.com/questions/5408/sensitivity-of-human-eye-to-luminance
        // the empirical exponent here is 0.33. I'm using a lower value to
        // compress the range more.
        a = Math.pow(a, 0.15);
        // Scale so that the RGB values fall in useful ranges, to look
        // vaguely like this:
        // https://www.scribd.com/document/410753432/Steel-Temp-Color-Chart
        a = a * 90;
        if (DEBUG)
            System.out.printf("r %f g %f b %f a %f\n", r, g, b, a);
        // relative luminance is always around 1
        // https://en.wikipedia.org/wiki/Relative_luminance
        if (DEBUG)
            System.out.printf("relative luminance %f\n", r * 0.2126 + g * 0.7152 + b * 0.0722);

        Color8Bit color = new Color8Bit((int) (r * a), (int) (g * a), (int) (b * a));
        if (DEBUG)
            System.out.printf("color %s\n", color);
        return color;
    }

    private static double r(double T) {
        double u = 0.000536332 * T;
        return 0.638749 + (u + 1.57533) / (u * u + 0.28664);
    }

    private static double g(double T) {
        double u = 0.0019639 * T;
        return 0.971029 + (u - 10.8015) / (u * u + 6.59002);
    }

    private static double b(double T) {
        double p = 0.00668406 * T + 23.3962;
        double u = 0.000941064 * T;
        double q = u * u + 0.00100641 * T + 10.9068;
        return 2.25398 - p / q;
    }

}
