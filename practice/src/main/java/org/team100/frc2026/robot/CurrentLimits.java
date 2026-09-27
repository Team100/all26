package org.team100.frc2026.robot;

import org.team100.lib.config.CurrentLimit;

/**
 * for background on drive current limits:
 * 
 * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
 * https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
 * https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
 * https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java
 * https://www.chiefdelphi.com/t/breaker-issues-in-our-strat-during-competition/516964/15
 * https://www.chiefdelphi.com/t/severe-voltage-drop-swerve-steer-jitter-with-ctre-swerve/481104/27
 */
public class CurrentLimits {
    public static final CurrentLimit INTAKE = new CurrentLimit(50, 30);
    public static final CurrentLimit INTAKE_EXTEND = new CurrentLimit(20, 40);
    public static final CurrentLimit CONVEYOR = new CurrentLimit(50, 30);
    public static final CurrentLimit FEEDER = new CurrentLimit(50, 30);
    public static final CurrentLimit SHOOTER = new CurrentLimit(60, 80);
    public static final CurrentLimit SHOOTER_HOOD = new CurrentLimit(50, 30);
    public static final CurrentLimit DRIVE = new CurrentLimit(90, 70);
    public static final CurrentLimit STEERING = new CurrentLimit(60, 30);
}
