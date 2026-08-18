package org.team100.lib.subsystems.tank;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;

public class TankControllerTest {

    /**
     * The LTVUnicycleController just multiplies a constant K times the position
     * error:
     * 
     * u = Ke
     * 
     * But it's clever in creating K.
     * 
     * So what is K? It makes a lookup table of K matrices, indexed by velocity.
     * 
     * The answer (remember, all robot-relative):
     * 
     * X feedback is just proportional to x error
     * Rotation feedback is proporional to a mix of y error and rotation error
     * 
     * The mix favors rotation more when going faster, which makes sense: the effect
     * of going "the wrong way" is greater when the velocity is higher.
     */
    @Test
    void testWhatIsK() {
        LTVUnicycleController controller = new LTVUnicycleController(0.020);
        // the K varies by linear velocity (3rd arg here)
        // motionless:
        controller.calculate(new Pose2d(), new Pose2d(), 0, 0);
        // K = [13, 0, 0]
        // ....[ 0, 15, 1]
        // so U_X is ~ x error
        // and U_R is ~ mix of (mostly) y err and r err

        controller = new LTVUnicycleController(0.020);
        // fast:
        controller.calculate(new Pose2d(), new Pose2d(), 5, 0);
        // K = [14, 0, 0]
        // ....[0, 14, 10]
        // so U_X is still ~ x error
        // and U_R is a now ~ a more even mix of y err and r err
    }

}
