package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.util.Color8Bit;

public class TannerHellandTest {
    @Test
    void test0() {
        // should be black, but is red.
        assertEquals(
                new Color8Bit(255, 0, 0),
                TannerHelland.color(100));
    }

    @Test
    void test1() {
        // just above black
        assertEquals(
                new Color8Bit(255, 39, 0),
                TannerHelland.color(750));
    }

    @Test
    void test2() {
        // deep red
        assertEquals(
                new Color8Bit(255, 67, 0),
                TannerHelland.color(1000));
    }

    @Test
    void test3() {
        // orange
        assertEquals(
                new Color8Bit(255, 123, 0),
                TannerHelland.color(1750));
    }

    @Test
    void test4() {
        // warm white
        assertEquals(
                new Color8Bit(255, 159, 70),
                TannerHelland.color(2500));
    }

    @Test
    void test5() {
        // design temperature
        assertEquals(
                new Color8Bit(255, 183, 123),
                TannerHelland.color(3200));
    }
}
