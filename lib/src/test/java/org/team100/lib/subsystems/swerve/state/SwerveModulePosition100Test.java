package org.team100.lib.subsystems.swerve.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.subsystems.swerve.kinodynamics.struct.SwerveModulePosition100Struct;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDelta;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;

import edu.wpi.first.math.geometry.Rotation2d;

class SwerveModulePosition100Test {
    private static final double DELTA = 0.001;

    @Test
    void testStruct() {
        SwerveModulePosition100Struct s = SwerveModulePosition100.struct;
        ByteBuffer bb = ByteBuffer.allocate(s.getSize());
        SwerveModulePosition100 p = new SwerveModulePosition100(1, Optional.of(Rotation2d.kZero));
        s.pack(bb, p);
        assertEquals(0, bb.remaining());
        // distance = 8
        // boolean = 1
        // angle = 8
        assertEquals(17, bb.position());
        bb.rewind();
        SwerveModulePosition100 p2 = s.unpack(bb);
        assertEquals(1.0, p2.distanceMeters(), 0.001);
        assertEquals(0.0, p2.unwrappedAngle().get().getRadians(), 0.001);
    }

    @Test
    void testEmpty() {
        SwerveModulePosition100Struct s = SwerveModulePosition100.struct;
        ByteBuffer bb = ByteBuffer.allocate(s.getSize());
        SwerveModulePosition100 p = new SwerveModulePosition100(1, Optional.empty());
        s.pack(bb, p);
        assertEquals(0, bb.remaining());
        assertEquals(17, bb.position());
        bb.rewind();
        SwerveModulePosition100 p2 = s.unpack(bb);
        assertEquals(1.0, p2.distanceMeters(), 0.001);
        assertTrue(p2.unwrappedAngle().isEmpty());
    }

    @Test
    void testPlus() {
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModuleDelta delta = new SwerveModuleDelta(
                    0, Optional.of(Rotation2d.kZero));
            SwerveModulePosition100 result = start.plus(delta);
            assertEquals(0, result.distanceMeters(), DELTA);
            assertEquals(0, result.unwrappedAngle().get().getDegrees(), DELTA);
        }
        {
            SwerveModulePosition100 start = new SwerveModulePosition100(
                    1, Optional.of(Rotation2d.fromDegrees(2)));
            SwerveModuleDelta delta = new SwerveModuleDelta(
                    3, Optional.of(Rotation2d.fromDegrees(4)));
            SwerveModulePosition100 result = start.plus(delta);
            assertEquals(4, result.distanceMeters(), DELTA);
            assertEquals(4, result.unwrappedAngle().get().getDegrees(), DELTA);
        }

    }
}
