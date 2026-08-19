package org.team100.lib.util;

import org.junit.jupiter.api.Test;
import org.wpilib.math.linalg.MatBuilder;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Nat;

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
