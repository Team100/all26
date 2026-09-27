package org.team100.frc2026.auton;

import org.wpilib.math.geometry.Translation2d;


public class GraphAvoid {
    private final Translation2d point;

        public GraphAvoid(Translation2d point) {
            this.point = point;
        }

        public Translation2d getPoint () {
            return point;

    
        }
    }

  