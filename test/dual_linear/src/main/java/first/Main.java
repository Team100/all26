
package first;

import org.wpilib.framework.RobotBase;

import first.robot.Robot;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
