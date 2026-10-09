package org.team100.frc2026.auton;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BumpZones {
    public static final Rectangle2d BLUE_BUMP_LEFT = 
        new Rectangle2d(
            new Translation2d(5.2, 6.43), 
            new Translation2d(4.1, 4.6)
        );

    public static final Rectangle2d BLUE_BUMP_RIGHT = 
        new Rectangle2d(
            new Translation2d(5.2, 3.2), 
            new Translation2d(4.1, 1.64)
        );

    public static final Rectangle2d RED_BUMP_RIGHT = 
        new Rectangle2d(
            new Translation2d(12.5, 6.43), 
            new Translation2d(11.32, 4.6)
        );
    
    public static final Rectangle2d RED_BUMP_LEFT = 
        new Rectangle2d(
            new Translation2d(12.5, 3.2), 
            new Translation2d(11.32, 1.64)
        );
        
}
