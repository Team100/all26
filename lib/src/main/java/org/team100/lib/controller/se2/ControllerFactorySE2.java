package org.team100.lib.controller.se2;

import org.team100.lib.logging.LoggerFactory;

/**
 * Known-good controller settings.
 */
public class ControllerFactorySE2 {

    public static FullStateControllerSE2 compbot(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                2.9, // P for x/y
                3.5, // P for theta
                0.025, // P for v
                0.01, // P for omega
                0.02, // x tolerance
                0.3, // theta tolerance
                1, // v tolerance
                1);// omega tolerance
    }

    public static FullStateControllerSE2 swerve1(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                2, // P for x/y
                2, // P for theta
                0.01, // P for v
                0, // P for omega
                0.001, // x tolerance
                0.01, // theta tolerance
                1, // v tolerance
                1);// omega tolerance
    }

    public static FullStateControllerSE2 swerve2(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                4, // P for x/y
                4, // P for theta
                0.25, // P for v
                0.25, // P for omega
                0.01, // x tolerance
                0.02, // theta tolerance
                0.01, // v tolerance
                0.02); // omega tolerance
    }

    public static FullStateControllerSE2 rookiebot(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                3, // P for x/y
                3.5, // P for theta
                0.05, // P for v
                0, // P for omega
                0.01, // x tolerance
                0.01, // theta tolerance
                1, // v tolerance
                1); // omega tolerance
    }

    public static FullStateControllerSE2 betabot(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                3, // for x/y
                3.5, // P for theta
                0.05, // P for v
                0, // P for omega
                0.025, // x tolerance
                0.025, // theta tolerance
                1, // v tolerance
                1); // omega tolerance
    }

    public static FullStateControllerSE2 sim(LoggerFactory log) {
        // this is for simulation, don't use these values
        return new FullStateControllerSE2(log, 3.0, 3.5, 0.05, 0, 0.05, 0.05, 1, 1);
    }

    public static FullStateControllerSE2 ridiculous(LoggerFactory log) {
        return new FullStateControllerSE2(log, 3, 3, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01);
    }

    public static FullStateControllerSE2 fieldRelativeFancyPIDF(LoggerFactory log) {
        return new FullStateControllerSE2(log, 2.4, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerSE2 fieldRelativeGoodPIDF(LoggerFactory log) {
        return new FullStateControllerSE2(log, 1, 1.3, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerSE2 autoFieldRelativePIDF(LoggerFactory log) {
        return new FullStateControllerSE2(log, 1.5, 1.3, 0, 0, 0.1, 0.1, 0.1, 0.1);
    }

    public static FullStateControllerSE2 auto2025LooseTolerance(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                7.2, // p cartesian
                3.5, // p theta
                0.055, // p cartesian v
                0.01, // p theta v
                0.035, // x tol
                0.1, // theta tol
                1, // xdot tol
                1); // omega tol
    }

    public static FullStateControllerSE2 pick(LoggerFactory log) {
        return new FullStateControllerSE2(log,
                7.2, // p cartesian
                3.5, // p theta
                0.055, // p cartesian v
                0.01, // p theta v
                0.15, // x tol
                0.1, // theta tol
                4, // xdot tol
                4); // omega tol
    }

    ////////////////
    //
    // don't use these for real robots
    //

    public static FullStateControllerSE2 testFieldRelativePIDF(LoggerFactory log) {
        return new FullStateControllerSE2(log, 2.4, 2.4, 0.1, 0.1, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerSE2 testFieldRelativeFFOnly(LoggerFactory log) {
        return new FullStateControllerSE2(log, 0, 0, 0, 0, 0.01, 0.02, 0.01, 0.02);
    }

    public static FullStateControllerSE2 test(LoggerFactory log) {
        return new FullStateControllerSE2(log, 3.0, 3.5, 0, 0, 0.01, 0.01, 0.01, 0.01);
    }

    /** high gains used in tests. */
    public static FullStateControllerSE2 test2(LoggerFactory log) {
        return new FullStateControllerSE2(log, 4, 4, 0.25, 0.25, 0.01, 0.02, 0.01, 0.02);
    }

    private ControllerFactorySE2() {
        // don't call this
    }

}
