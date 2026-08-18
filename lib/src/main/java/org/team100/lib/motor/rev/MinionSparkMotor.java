package org.team100.lib.motor.rev;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Minion motor on the sparkmax controller
 * 
 * @see https://store.ctr-electronics.com/products/minion-brushless-motor
 */
public class MinionSparkMotor extends CANSparkMotor {
    /**
     * See SparkMaxConfig.Presets.CTRE_Minion
     * https://www.chiefdelphi.com/t/minion-motors-on-sparkmax/518200/2
     * This value appears to make the motor run backwards
     */
    // private static final double COMMUTATION_DEGREES = 60;
    /** This value appears to work correctly. */
    private static final double COMMUTATION_DEGREES = -120;

    public MinionSparkMotor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            CurrentLimit limit,
            Friction friction,
            PIDConstants pid,
            int averageDepth,
            int measurementPeriod) {
        super(parent, currentLog,
                new SparkMax(canId.id, MotorType.kBrushless),
                neutral, motorPhase, limit, friction, pid,
                COMMUTATION_DEGREES, averageDepth, measurementPeriod,
                false);
    }

    @Override
    public double kROhms() {
        return 0.056;
    }

    @Override
    public double kTNm_amp() {
        return 0.015;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 7700;
    }
}
