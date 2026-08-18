package org.team100.lib.motor.rev;

import org.team100.lib.config.CurrentLimit;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Neo Vortex motor.
 * 
 * @see https://www.revrobotics.com/rev-21-1652/
 */
public class NeoVortexCANSparkMotor extends CANSparkMotor {
    public NeoVortexCANSparkMotor(
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
                new SparkFlex(canId.id, MotorType.kBrushless),
                neutral, motorPhase, limit, friction, pid,
                0, averageDepth, measurementPeriod, true);
    }

    @Override
    public double kROhms() {
        return 0.057;
    }

    @Override
    public double kTNm_amp() {
        return 0.017;
    }

    @Override
    public double kFreeSpeedRPM() {
        // return Double.MAX_VALUE;
        return 6784;
    }
}
