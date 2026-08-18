package org.team100.frc2026.targeting;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.targeting.FiringParameters;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Chooses where to aim, supplies parameters.
 * 
 * Data here
 * https://docs.google.com/spreadsheets/d/14-wiGWctZqFQl5EalOh9EtM_RuiWSOV8aGvHj-KpQWs/edit?gid=0#gid=0
 * 
 */
public class Targeter {
    private final Supplier<Translation2d> m_position;
    /** High shots to score */
    private final ShooterTable m_score;
    /** Low shots to save power */
    private final ShooterTable m_lob;

    public Targeter(Supplier<Translation2d> position) {
        m_position = position;
        // from 3/12/26
        m_score = new ShooterTable(
                List.of(
                        // this is close to the minimum possible so it's ok for it to be the edge value
                        new FiringParameters(1.65, 12.00, 0.00, 0.787),
                        new FiringParameters(2.28, 14.00, 0.00, 0.964),
                        new FiringParameters(2.99, 14.00, 0.15, 0.845),
                        new FiringParameters(3.52, 15.00, 0.20, 0.912),
                        new FiringParameters(4.30, 17.00, 0.35, 0.845),
                        // this is close to the maximum range possible (in the corner) so it's ok for it
                        // to be the edge value.
                        new FiringParameters(5.32, 19.00, 0.30, 1.032),
                        // whoops, when moving fast, longer range is required
                        // so just guess.
                        // TODO: measure this
                        new FiringParameters(7, 21.00, 0.30, 1.4)
                //
                ));

        // from 3/12/26
        m_lob = new ShooterTable(
                List.of(
                        // this is to make the lookup never fail
                        new FiringParameters(0.00, 17.00, 0.25, 1.268),
                        // this is the actual example we ran
                        new FiringParameters(6.00, 17.00, 0.25, 1.268),
                        // this is to make the lookup never fail
                        new FiringParameters(16.00, 17.00, 0.25, 1.268)));
    }

    /**
     * Firing parameters for the specified range.
     * Empty if there is no valid solution for that range (e.g. it's too far or too
     * close)
     * 
     * TODO: extract target selection and shot type
     */
    public Optional<FiringParameters> forRange(double rangeM) {
        Translation2d p = m_position.get();
        if (FieldConstants2026.isInAllianceZone(p)) {
            return m_score.forRange(rangeM);
        }
        return m_lob.forRange(rangeM);
    }

}
