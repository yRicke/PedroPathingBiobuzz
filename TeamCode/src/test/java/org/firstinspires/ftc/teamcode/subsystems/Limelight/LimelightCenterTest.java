package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class LimelightCenterTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void eachOfSixteenTagsAloneFindsSameCenterForItsCell() {
        int[] ids = {30, 31, 32, 33, 34, 35, 36, 37,
                38, 39, 40, 41, 42, 43, 44, 45};
        double[] tagX = {0.1349, 0.23015, 0.36985, 0.4651};

        for (int i = 0; i < ids.length; i++) {
            FiducialResult tag = tag(ids[i], tagX[i % 4], -0.4, 2, 0, 0, 0, 1);
            int firstTag = 30 + (ids[i] - 30) / 4 * 4;

            AprilTagObservation result =
                    estimate(Arrays.asList(tag), firstTag);

            assertTrue("Tag " + ids[i], result.hasTarget());
            assertEquals(8.530765609948133, result.getTxDegrees(), EPSILON);
            assertEquals(1, result.getTagsUsed());
        }
    }

    @Test
    public void hiddenRightmostTagDoesNotMoveEstimatedCenter() {
        AprilTagObservation result = estimate(
                Arrays.asList(tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                        tag(43, -0.06985, 0, 2, 0, 0, 0, 1),
                        tag(44, 0.06985, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.hasTarget());
        assertEquals(0, result.getTxDegrees(), EPSILON);
        assertEquals(3, result.getTagsUsed());
    }

    @Test
    public void areaWeightedFusionCombinesCenterPositionsRatherThanTagCenters() {
        AprilTagObservation result = estimate(
                Arrays.asList(tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                        tag(45, 0.5651, 0, 2, 0, 0, 0, 3)), 42);

        assertTrue(result.hasTarget());
        assertEquals(8.530765609948133, result.getTxDegrees(), EPSILON);
        assertEquals(2, result.getTagsUsed());
    }

    @Test
    public void tagRotationProjectsOffsetWithDepthAndHeightChanges() {
        FiducialResult tilted = tag(42, 0.2285096029175946, -0.441275,
                2.142980794164811, 17, 60, 30, 1);

        AprilTagObservation result =
                estimate(Arrays.asList(tilted), 42);

        assertTrue(result.hasTarget());
        assertEquals(8.530765609948133, result.getTxDegrees(), EPSILON);
    }

    @Test
    public void upsideDownCameraPoseRotatesTheTagOffsetBeforeProjection() {
        FiducialResult rotated = tag(42, -0.1349, 0.4, 2, 0, 0, 180, 1);

        AprilTagObservation result =
                estimate(Arrays.asList(rotated), 42);

        assertTrue(result.hasTarget());
        assertEquals(-8.530765609948133, result.getTxDegrees(), EPSILON);
    }

    @Test
    public void redTiltedAndRotatedCellUsesLocalOffsetsWithoutAllianceSignFlip() {
        for (int first : new int[] {30, 34}) {
            // A mesma CELL vista girada 180 graus no plano da imagem.
            AprilTagObservation rotated = estimate(Arrays.asList(
                    tag(first, -0.1349, 0.4, 2, 0, 0, 180, 1)), first);
            assertEquals(-8.530765609948133, rotated.getTxDegrees(), EPSILON);
            // A inclinacao do HIVE e resolvida pela pose, nao por altura fixa.
            AprilTagObservation tilted = estimate(Arrays.asList(
                    tag(first, 0.2285096029175946, -0.441275,
                            2.142980794164811, 17, 60, 30, 1)), first);
            assertEquals(8.530765609948133, tilted.getTxDegrees(), EPSILON);
        }
    }

    @Test
    public void distanceUnitIsConvertedBeforeApplyingMetricGeometry() {
        FiducialResult inches = mock(FiducialResult.class);
        when(inches.getFiducialId()).thenReturn(42);
        when(inches.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, -6.5, 0, 40, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));

        AprilTagObservation result =
                estimate(Arrays.asList(inches), 42);

        assertTrue(result.hasTarget());
        assertEquals(0, result.getTxDegrees(), EPSILON);
    }

    @Test
    public void otherHiveDoesNotContributeToEstimate() {
        assertFalse(estimate(
                Arrays.asList(tag(38, 0, 0, 2, 0, 0, 0, 1)), 42).hasTarget());
    }

    @Test
    public void missingZeroBehindOrNonFinitePoseCannotProduceHeading() {
        FiducialResult missing = mock(FiducialResult.class);
        when(missing.getFiducialId()).thenReturn(43);

        assertFalse(estimate(Arrays.asList(missing,
                tag(43, 0, 0, 0, 0, 0, 0, 1),
                tag(43, 0, 0, -2, 0, 0, 0, 1),
                tag(43, Double.NaN, 0, 2, 0, 0, 0, 1),
                tag(43, 0, 0, 2, 0, Double.NaN, 0, 1)), 42).hasTarget());
    }

    @Test
    public void invalidTagDoesNotDiscardAnotherUsableTag() {
        AprilTagObservation result = estimate(
                Arrays.asList(tag(42, 0, 0, 0, 0, 0, 0, 1),
                        tag(43, -0.06985, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.hasTarget());
        assertEquals(1, result.getTagsUsed());
    }

    @Test
    public void eachTagUsesTheSameThreeDimensionalCenterDistance() {
        double[] tagX = {2.8349, 2.93015, 3.06985, 3.1651};
        for (int id = 30; id <= 45; id++) {
            int firstTag = 30 + (id - 30) / 4 * 4;
            Limelight result = read(Arrays.asList(
                    tag(id, tagX[(id - 30) % 4], 4, 12, 0, 0, 0, 1)), firstTag);
            // Centro (3, 4, 12): distancia 13 m, incluindo altura e offset local.
            assertEquals("Tag " + id, 13, result.getDistanceMeters().getAsDouble(), EPSILON);
        }
    }

    @Test
    public void tiltedAndInvertedPosesKeepTheSameCenterDistance() {
        for (int first : new int[] {30, 34, 38, 42}) {
            Limelight tilted = read(Arrays.asList(tag(first,
                    0.2285096029175946, -0.441275, 2.142980794164811,
                    17, 60, 30, 1)), first);
            Limelight inverted = read(Arrays.asList(tag(first,
                    -0.1349, 0.4, 2, 0, 0, 180, 1)), first);
            // Ambos representam o centro a (+/-0.3, +/-0.4, 2) m.
            assertEquals(2.0615528128088303, tilted.getDistanceMeters().getAsDouble(), EPSILON);
            assertEquals(2.0615528128088303, inverted.getDistanceMeters().getAsDouble(), EPSILON);
        }
    }

    @Test
    public void distanceUsesTheAreaWeightedCenterIncludingHeight() {
        Limelight result = read(Arrays.asList(
                tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                tag(45, 0.5651, 0.8, 2, 0, 0, 0, 3)), 42);
        // Centro ponderado (0.3, 0.6, 2), nao media das distancias individuais.
        assertEquals(2.1095023109728985, result.getDistanceMeters().getAsDouble(), EPSILON);
    }

    @Test
    public void hiddenTagsDoNotChangeTheDistanceReference() {
        Limelight result = read(Arrays.asList(
                tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                tag(43, -0.06985, 0, 2, 0, 0, 0, 1),
                tag(44, 0.06985, 0, 2, 0, 0, 0, 1)), 42);
        assertEquals(2, result.getDistanceMeters().getAsDouble(), EPSILON);
    }

    @Test
    public void nonFiniteHeightAndCenterBehindCameraCannotSupplyDistance() {
        for (FiducialResult invalid : Arrays.asList(
                tag(42, 0, Double.NaN, 2, 0, 0, 0, 1),
                tag(42, 0, Double.POSITIVE_INFINITY, 2, 0, 0, 0, 1),
                tag(42, 0, 0, 0.01, 0, 90, 0, 1))) {
            Limelight result = read(Arrays.asList(invalid), 42);
            assertFalse(result.getDistanceMeters().isPresent());
        }
    }

    private AprilTagObservation estimate(List<FiducialResult> tags, int firstTagId) {
        return read(tags, firstTagId).getObservation();
    }

    private Limelight read(List<FiducialResult> tags, int firstTagId) {
        Limelight3A camera = mock(Limelight3A.class);
        LLResult frame = mock(LLResult.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        when(camera.getLatestResult()).thenReturn(frame);
        when(frame.getTimestamp()).thenReturn(1.0);
        when(frame.getPipelineIndex()).thenReturn(8);
        when(frame.isValid()).thenReturn(true);
        when(frame.getFiducialResults()).thenReturn(tags);
        Limelight limelight = new Limelight(camera, null, false, 8, System::nanoTime);
        limelight.run(firstTagId == 34 || firstTagId == 38,
                firstTagId == 30 || firstTagId == 42, firstTagId >= 38, firstTagId < 38);
        return limelight;
    }

    private FiducialResult tag(int id, double x, double y, double z,
                               double roll, double pitch, double yaw, double area) {
        FiducialResult tag = mock(FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetArea()).thenReturn(area);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.METER, x, y, z, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0)));
        return tag;
    }
}
