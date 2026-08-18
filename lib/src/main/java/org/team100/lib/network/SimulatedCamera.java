package org.team100.lib.network;

import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.localization.Blip;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;

/**
 * Uses a function to supply ground-truth values for a time in the somewhat
 * distant past, and publishes it on network tables with an appropriate
 * timestamp.
 */
public class SimulatedCamera implements Runnable {
    /** Measured camera delay in seconds, see studies/camera_delay. */
    private static final double DELAY_S = 0.085;

    private final DoubleFunction<Rotation2d> m_truth;

    /** Client instance, not the default. */
    private final NetworkTableInstance m_inst;
    private final StructArrayPublisher<Blip> m_pub;

    public SimulatedCamera(DoubleFunction<Rotation2d> truth) {
        m_truth = truth;
        m_inst = NetworkTableInstance.create();
        // This is a client just like the camera is a client.
        m_inst.setServer("localhost");
        m_inst.startClient4("SimulatedCamera");
        m_pub = m_inst.getStructArrayTopic("vision/0/blips", Blip.struct)
                .publish(PubSubOption.keepDuplicates(true));
    }

    @Override
    public void run() {
        // Sample the ground-truth value in the past.
        double past = Takt.actual() - DELAY_S;
        Rotation2d pastValue = m_truth.apply(past);

        // Camera coordinates are z-forward so roll axis appears there.
        Transform3d t = new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, 0, pastValue.getRadians()));
        // Past time in microseconds.
        long pastUs = (long) (past * 1000000.0);

        Blip b = new Blip(pastUs, 1, t);

        m_pub.set(new Blip[] { b }, pastUs);
        m_inst.flushLocal();
    }

}
