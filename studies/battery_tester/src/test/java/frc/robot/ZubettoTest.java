package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.util.Color8Bit;

public class ZubettoTest {
    @Test
    void test0() {
        // black
        assertEquals(
                new Color8Bit(0, 0, 0),
                Zubetto.color(100));
    }

    @Test
    void test1() {
        // just above black
        assertEquals(
                new Color8Bit(47, 0, 0),
                Zubetto.color(750));
    }

    @Test
    void test2() {
        // deep red
        assertEquals(
                new Color8Bit(147, 4, 0),
                Zubetto.color(1000));
    }

    @Test
    void test3() {
        // orange
        assertEquals(
                new Color8Bit(255, 101, 0),
                Zubetto.color(1750));
    }

    @Test
    void test4() {
        // warm white
        assertEquals(
                new Color8Bit(255, 255, 47),
                Zubetto.color(2500));
    }

    @Test
    void test5() {
        // design temperature
        assertEquals(
                new Color8Bit(255, 255, 156),
                Zubetto.color(3200));
    }
}
