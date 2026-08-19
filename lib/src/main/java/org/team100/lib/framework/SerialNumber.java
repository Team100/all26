package org.team100.lib.framework;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.wpilib.system.RobotController;

/**
 * Systemcore does not support serialnumber,
 * so this is a workaround.
 * see https://github.com/wpilibsuite/SystemcoreTesting/issues/38
 */
public class SerialNumber {

    public static String get() {
        String s = RobotController.getSerialNumber();
        if (s.isEmpty()) {
            try {
                // this file contains a null terminator, so remove it.
                return Files.readString(Path.of("/sys/firmware/devicetree/base/serial-number")).replace("\0", "");
            } catch (IOException e) {
                return "";
            }
        }
        return s;
    }
}
