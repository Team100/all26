package org.team100.lib.config;

/**
 * The stator limit is proportional to the applied torque. Use this to regulate
 * the force produced by the motor. This is usually only useful when the motor
 * is stalled, e.g. the drive wheels in a pushing match, or a manipulator
 * holding on to something.
 * 
 * The supply limit is essentially a power limit, since the supply voltage is
 * (very roughly) constant. Use this to moderate the destruction of the robot
 * battery.
 * 
 * for background on drive current limits:
 * 
 * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
 * https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
 * https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
 * https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java
 * https://www.chiefdelphi.com/t/breaker-issues-in-our-strat-during-competition/516964/15
 * https://www.chiefdelphi.com/t/severe-voltage-drop-swerve-steer-jitter-with-ctre-swerve/481104/27
 * 
 * @param stator limit in amps
 * @param supply limit in amps
 */
public record CurrentLimit(double stator, double supply) {
}
