package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.se2.VelocitySE2;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class OffsetUtil {

    /**
     * The perpendicular component of v across r, as an angular velocity.
     * 
     * omega = (r \cross v) / r^2
     * 
     * Cartesian components are always zero.
     */
    static VelocitySE2 omega(Vector<N3> r, Vector<N3> v) {
        return VelocitySE2.fromVector(
                Vector.cross(r, v).div(r.norm() * r.norm()));
    }

    /**
     * Computes the cartesian velocity created by the rotational velocity omega,
     * through the radius r.
     * 
     * v = omega \cross r
     * 
     * Omega component is always zero.
     */
    static VelocitySE2 tangentialVelocity(
            Vector<N3> omega, Vector<N3> r) {
        return VelocitySE2.fromVector(
                Vector.cross(omega, r));
    }

    /**
     * Cartesian component of velocity.
     */
    static Vector<N3> velocity(VelocitySE2 v) {
        return v.vVector();
    }

    /**
     * Omega component of the velocity
     */
    static Vector<N3> omega(VelocitySE2 v) {
        return v.omegaVector();
    }

}
