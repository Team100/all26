package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class MatUtilTest {
    @Test
    void test0() {
        Matrix<N2, N2> m = MatBuilder.fill(Nat.N2(), Nat.N2(), //
                1, 2, //
                3, 4);
        System.out.println(m.diag());
    }

    @Test
    void test1() {
        Vector<N2> m = VecBuilder.fill(1, 2);
        System.out.println(m.diag());
    }
}
