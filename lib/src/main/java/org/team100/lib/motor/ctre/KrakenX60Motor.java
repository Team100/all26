package org.team100.lib.motor.ctre;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

/**
 * Kraken X60 using Phoenix 6.
 * 
 * https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
 */
public class KrakenX60Motor extends Talon6Motor {

    public KrakenX60Motor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase phase,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid) {
        super(parent, currentLog, canId, neutral, phase, limit, friction, pid);
    }

    @Override
    public double R() {
        // 12.0 V, 483 A
        return 0.025;
    }

    @Override
    public double kT() {
        // 9.39 Nm, 483 A
        return 0.019;
    }

    @Override
    public double kE() {
        // 60 * 12 / (5800 * 2 * pi) volt-sec/rad
        return 0.0197572;
    }
}
