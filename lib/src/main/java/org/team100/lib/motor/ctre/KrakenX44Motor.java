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
 * Kraken X44 using Phoenix 6.
 * 
 * https://docs.wcproducts.com/welcome/electronics/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance
 */
public class KrakenX44Motor extends Talon6Motor {

    public KrakenX44Motor(
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
        // 12.0 V, 279 A
        return 0.043;
    }

    @Override
    public double kT() {
        // 4.11 Nm, 279 A
        return 0.015;
    }

    @Override
    public double kE() {
        // 60 * 12 / (7530 * 2 * pi) volt-sec/rad
        return 0.0152180;
    }
}
