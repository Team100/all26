package org.team100.lib.localization.gtsam;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.localization.map.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.network.BlipWithCorners;
import org.team100.lib.network.CameraReader;
import org.wpilib.driverstation.Alliance;
import org.wpilib.util.struct.StructBuffer;

/**
 * Reads camera observations, produces corners for GTSAM
 */
public class CornerReader {
    private static final boolean DEBUG = true;
    private final CameraReader<BlipWithCorners> m_reader;
    private final TagToCorners m_translator;
    private final CornerFilter m_filter;

    public CornerReader(
            AprilTagFieldLayoutWithCorrectOrientation layout,
            Supplier<Optional<Alliance>> alliance) {
        m_reader = new CameraReader<>(
                "vision", "blips_with_corners",
                StructBuffer.create(BlipWithCorners.struct));
        m_translator = new TagToCorners(alliance, layout);
        m_filter = new CornerFilter();
    }

    public List<Corners> read() {
        return m_filter.filter(readAll());
    }

    public List<Corners> readAll() {
        List<CameraReader.Record<BlipWithCorners>> records = m_reader.getRecords();
        if (DEBUG)
            System.out.printf("#### CornerReader.readAll() records %d\n", records.size());
        List<Corners> measurements = new ArrayList<>();
        for (CameraReader.Record<BlipWithCorners> r : records) {
            measurements.addAll(m_translator.convert(r.camera(), r.values()));
        }
                if (DEBUG)
            System.out.printf("#### CornerReader.readAll() measurements %d\n", measurements.size());
        return measurements;
    }

}
