package org.team100.lib.targeting;

import java.util.List;
import java.util.Optional;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.VelocityR2Logger;
import org.team100.lib.logging.LoggerFactory.Translation2dLogger;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * See INTERCEPT.md for details.
 */
public class Intercept {
    private final Translation2dLogger m_logT0;
    private final VelocityR2Logger m_logVT;
    private final DoubleLogger m_logEta;

    public Intercept(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_logT0 = log.translation2dLogger(Level.TRACE, "T0");
        m_logVT = log.VelocityR2Logger(Level.TRACE, "vT");
        m_logEta = log.doubleLogger(Level.TRACE, "ETA");
    }

    /**
     * Find a turret rotation which will intercept the target. If more than one
     * solution is possible, choose the sooner one. If no solution is possible,
     * return Optional.empty().
     * 
     * @param robotPosition  field-relative robot position, meters
     * @param robotVelocity  field-relative robot velocity, meters/sec
     * @param targetPosition field-relative target position, meters
     * @param targetVelocity field-relative target velocity, meters/sec
     * @param muzzleSpeed    speed of the projectile, meters/sec
     * @return field-relative firing solution azimuth
     */
    public Optional<Rotation2d> intercept(
            Translation2d robotPosition,
            VelocityR2 robotVelocity,
            Translation2d targetPosition,
            VelocityR2 targetVelocity,
            double muzzleSpeed) {
        double T0x = targetPosition.getX() - robotPosition.getX();
        double T0y = targetPosition.getY() - robotPosition.getY();
        Translation2d T0 = new Translation2d(T0x, T0y);
        m_logT0.log(() -> T0);

        double vTx = targetVelocity.x() - robotVelocity.x();
        double vTy = targetVelocity.y() - robotVelocity.y();
        VelocityR2 vT = new VelocityR2(vTx, vTy);
        m_logVT.log(() -> vT);

        double T0_dot_vT = T0x * vTx + T0y * vTy;
        double vT_dot_vT = Math.pow(vTx, 2) + Math.pow(vTy, 2);

        double C = Math.pow(T0x, 2) + Math.pow(T0y, 2);

        double B = 2.0 * T0_dot_vT;
        double A = vT_dot_vT - Math.pow(muzzleSpeed, 2);
        List<Double> solutions = Math100.solveQuadratic(A, B, C);

        double bestTime = Double.MAX_VALUE;

        // Find the smallest positive time solution
        for (double t : solutions) {
            if (t >= 0 && t < bestTime) {
                bestTime = t;
            }
        }
        if (bestTime == Double.MAX_VALUE)
            return Optional.empty();

        double eta = bestTime;
        m_logEta.log(() -> eta);
        double Ix = T0x + vTx * eta;
        double Iy = T0y + vTy * eta;
        double theta = Math.atan2(Iy, Ix);
        return Optional.of(new Rotation2d(theta));
    }
}
