package org.team100.lib.spline.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.WaypointSE2;
import org.team100.lib.util.ChartUtil;

import org.wpilib.math.linalg.Vector;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.numbers.N2;

public class SplineSE2UtilTest {
    private static final double DELTA = 0.001;

    /**
     * We commonly want to draw the toolpoint path but actuate the mechanism path,
     * which is offset. If the offset is a pure rotation, then the path schedule can
     * be the same, but if the offset includes translation (e.g. for an extended
     * arm) then the mechanism (drivetrain) path is different, and needs to be
     * scheduled differently.
     * 
     * The curvature of the offset path is easy to compute from the derivatives of
     * the toolpoint path and the derivatives of the offset. Remember the offset
     * varies due to the rotation of the toolpoint, which is otherwise not included
     * in the curvature calculation.
     * 
     * An offset might have fixed length, or the length might vary, e.g. if an arm
     * is extended during the spline motion.
     */
    @Test
    void testOffset() {
        // toolpoint goes in a straight line but with rotation
        // so the offset (shown here with the vector) makes a quarter cyloid
        // velocity is +x the whole time
        // heading rate is -1 the whole time
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        SplineSE2 toolpoint = new SplineSE2(w0, w1);

        // the toolpoint path has no curvature
        for (double s = 0; s <= 1; s += 0.1) {
            // assertEquals(0, toolpoint.entry(s).point().getCurvatureRad_M(), DELTA);
            assertEquals(0, toolpoint.curvature(s), DELTA);
        }
        double length = 1.0;

        // initial offset is -y
        Vector<N2> or0 = SplineSE2Util.offsetR(toolpoint, length, 0);
        assertEquals(0, or0.get(0), DELTA);
        assertEquals(-1, or0.get(1), DELTA);
        // offset itself is rotating like heading (-theta)
        // though this is not the net motion of the offset endpoint
        Vector<N2> orp0 = SplineSE2Util.offsetRprime(toolpoint, length, 0);
        assertEquals(-1.111, orp0.get(0), DELTA);
        assertEquals(0, orp0.get(1), DELTA);

        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        List<VectorSeries> series = splineConverter.convert(List.of(toolpoint));

        // velocity vectors
        VectorSeries v = new VectorSeries("endpoints");
        for (double s = 0; s < 1.001; s += 0.05) {
            Vector<N2> p = SplineSE2Util.offsetR(toolpoint, length, s);
            Vector<N2> pprime = SplineSE2Util.offsetRprime(toolpoint, length, s);
            Vector<N2> rprime = toolpoint.rprime(s);
            double dx = pprime.get(0) + rprime.get(0);
            double dy = pprime.get(1) + rprime.get(1);
            v.add(p.get(0), p.get(1), dx * 0.05, dy * 0.05);
        }
        series.add(v);

        // acceleration vectors
        VectorSeries a = new VectorSeries("endpoints");
        for (double s = 0; s < 1.001; s += 0.05) {
            Vector<N2> p = SplineSE2Util.offsetR(toolpoint, length, s);
            Vector<N2> pprimeprime = SplineSE2Util.offsetRprimeprime(toolpoint, length, s);
            Vector<N2> rprimeprime = toolpoint.rprimeprime(s);
            double ax = pprimeprime.get(0) + rprimeprime.get(0);
            double ay = pprimeprime.get(1) + rprimeprime.get(1);
            a.add(p.get(0), p.get(1), ax * 0.05, ay * 0.05);
        }
        series.add(a);

        ChartUtil.plotOverlay(series, 500);
    }

    @Test
    void testTransform() {
        // Pose2d.transformBy() applies the transform in the pose frame.
        Pose2d p = new Pose2d(0, 0, Rotation2d.kCCW_Pi_2);
        Pose2d p2 = p.transformBy(new Transform2d(1, 0, new Rotation2d()));
        assertEquals(0, p2.getX(), DELTA);
        assertEquals(1, p2.getY(), DELTA);
        assertEquals(Rotation2d.kCCW_Pi_2.getRadians(), p2.getRotation().getRadians(), DELTA);
    }

}
