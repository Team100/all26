package org.team100.lib.localization;

public interface OdometryUpdaterInterface {
    void update();

    void replay(double sampleTime);
}
