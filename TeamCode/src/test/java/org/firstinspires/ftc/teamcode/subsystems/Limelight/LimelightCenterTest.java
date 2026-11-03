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

        for (int i = 0; i < ids.length; i++) {
            FiducialResult tag = tagForOpening(ids[i], 0.3, -0.4, 2, 0, 0, 0, 1);
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
                Arrays.asList(tagForOpening(42, 0, 0, 2, 0, 0, 0, 1),
                        tagForOpening(43, 0, 0, 2, 0, 0, 0, 1),
                        tagForOpening(44, 0, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.hasTarget());
        assertEquals(0, result.getTxDegrees(), EPSILON);
        assertEquals(3, result.getTagsUsed());
    }

    @Test
    public void areaWeightedFusionCombinesCenterPositionsRatherThanTagCenters() {
        AprilTagObservation result = estimate(
                Arrays.asList(tagForOpening(42, 0, 0, 2, 0, 0, 0, 1),
                        tagForOpening(45, 0.04, 0, 2, 0, 0, 0, 3)), 42);

        assertTrue(result.hasTarget());
        assertEquals(Math.toDegrees(Math.atan2(0.03, 2)), result.getTxDegrees(), EPSILON);
        assertEquals(2, result.getTagsUsed());
    }

    @Test
    public void tagRotationProjectsOffsetWithDepthAndHeightChanges() {
        FiducialResult tilted = tagForOpening(42, 0.3, -0.4, 2, 17, 60, 30, 1);

        AprilTagObservation result =
                estimate(Arrays.asList(tilted), 42);

        assertTrue(result.hasTarget());
        assertEquals(8.530765609948133, result.getTxDegrees(), EPSILON);
    }

    @Test
    public void upsideDownCameraPoseRotatesTheTagOffsetBeforeProjection() {
        FiducialResult rotated = tagForOpening(42, -0.3, 0.4, 2, 0, 0, 180, 1);

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
                    tagForOpening(first, -0.3, 0.4, 2, 0, 0, 180, 1)), first);
            assertEquals(-8.530765609948133, rotated.getTxDegrees(), EPSILON);
            // A inclinacao do HIVE e resolvida pela pose, nao por altura fixa.
            AprilTagObservation tilted = estimate(Arrays.asList(
                    tagForOpening(first, 0.3, -0.4, 2, 17, 60, 30, 1)), first);
            assertEquals(8.530765609948133, tilted.getTxDegrees(), EPSILON);
        }
    }

    @Test
    public void distanceUnitIsConvertedBeforeApplyingMetricGeometry() {
        FiducialResult inches = mock(FiducialResult.class);
        when(inches.getFiducialId()).thenReturn(42);
        when(inches.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, -6.5, 7.1874, 40 - 5.622, 0),
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
                        tagForOpening(43, 0, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.hasTarget());
        assertEquals(1, result.getTagsUsed());
    }

    @Test
    public void eachTagUsesTheSameThreeDimensionalCenterDistance() {
        for (int id = 30; id <= 45; id++) {
            int firstTag = 30 + (id - 30) / 4 * 4;
            Limelight result = read(Arrays.asList(
                    tagForOpening(id, 3, 4, 12, 0, 0, 0, 1)), firstTag);
            // Centro (3, 4, 12): distancia 13 m, incluindo altura e offset local.
            assertEquals("Tag " + id, 13, result.getDistanceMeters().getAsDouble(), EPSILON);
        }
    }

    @Test
    public void tiltedAndInvertedPosesKeepTheSameCenterDistance() {
        for (int first : new int[] {30, 34, 38, 42}) {
            Limelight tilted = read(Arrays.asList(tagForOpening(first,
                    0.3, -0.4, 2, 17, 60, 30, 1)), first);
            Limelight inverted = read(Arrays.asList(tagForOpening(first,
                    -0.3, 0.4, 2, 0, 0, 180, 1)), first);
            // Ambos representam o centro a (+/-0.3, +/-0.4, 2) m.
            assertEquals(2.0615528128088303, tilted.getDistanceMeters().getAsDouble(), EPSILON);
            assertEquals(2.0615528128088303, inverted.getDistanceMeters().getAsDouble(), EPSILON);
        }
    }

    @Test
    public void distanceUsesTheAreaWeightedCenterIncludingHeight() {
        Limelight result = read(Arrays.asList(
                tagForOpening(42, 0, 0, 2, 0, 0, 0, 1),
                tagForOpening(45, 0.04, 0.08, 2, 0, 0, 0, 3)), 42);
        // Pequeno ruido de pose: centro ponderado (0.03, 0.06, 2).
        assertEquals(Math.sqrt(4 + 0.03 * 0.03 + 0.06 * 0.06),
                result.getDistanceMeters().getAsDouble(), EPSILON);
    }

    @Test
    public void hiddenTagsDoNotChangeTheDistanceReference() {
        Limelight result = read(Arrays.asList(
                tagForOpening(42, 0, 0, 2, 0, 0, 0, 1),
                tagForOpening(43, 0, 0, 2, 0, 0, 0, 1),
                tagForOpening(44, 0, 0, 2, 0, 0, 0, 1)), 42);
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

    @Test
    public void oneLargeBadPoseCannotPullTheOpeningAwayFromThreeAgreeingTags() {
        AprilTagObservation result = estimate(Arrays.asList(
                tagForOpening(42, 0, -0.4, 2, 17, 60, 30, 1),
                tagForOpening(43, 0, -0.4, 2, 17, 60, 30, 1),
                tagForOpening(44, 0, -0.4, 2, 17, 60, 30, 1),
                tagForOpening(45, 0.5, -0.4, 2, 17, 60, 30, 90)), 42);

        assertTrue(result.hasTarget());
        assertEquals(0, result.getTxDegrees(), EPSILON);
        assertEquals(3, result.getTagsUsed());
    }

    @Test
    public void conflictingPosesWithoutAMajorityCannotDefineTheOpening() {
        AprilTagObservation result = estimate(Arrays.asList(
                tagForOpening(42, -0.5, 0, 2, 0, 0, 0, 1),
                tagForOpening(43, 0.5, 0, 2, 0, 0, 0, 10)), 42);

        assertFalse(result.hasTarget());
    }

    private Limelight read(List<FiducialResult> tags, int firstTagId) {
        return read(tags, firstTagId, new AimGeometry(0, 0, 0, 0, 0, 0));
    }

    private Limelight read(List<FiducialResult> tags, int firstTagId, AimGeometry geometry) {
        Limelight3A camera = mock(Limelight3A.class);
        LLResult frame = mock(LLResult.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        when(camera.getLatestResult()).thenReturn(frame);
        when(frame.getTimestamp()).thenReturn(1.0);
        when(frame.getPipelineIndex()).thenReturn(8);
        when(frame.isValid()).thenReturn(true);
        when(frame.getFiducialResults()).thenReturn(tags);
        Limelight limelight = new Limelight(camera, null, geometry, 8, System::nanoTime);
        limelight.run(firstTagId == 34 || firstTagId == 38,
                firstTagId == 30 || firstTagId == 42, firstTagId >= 38, firstTagId < 38);
        return limelight;
    }

    @Test
    public void calibratedInvertedCameraKeepsTheOpeningFromBothSidesNearAndFarWithOcclusion() {
        AimGeometry mount = new AimGeometry(0, 12.9, -10.691, 49.265, 0, 180);
        double pitch = Math.toRadians(49.265);
        for (int first : new int[] {30, 34, 38, 42}) {
            for (double x : new double[] {-0.5, 0, 0.5}) {
                for (double z : new double[] {1, 3.5}) {
                    // Abertura conhecida no lancador: (x, -1, z). Projecao inversa da montagem.
                    double y = -1 + 0.129, depth = z + 0.10691;
                    double cx = -x;
                    double cy = -(Math.cos(pitch) * y + Math.sin(pitch) * depth);
                    double cz = -Math.sin(pitch) * y + Math.cos(pitch) * depth;
                    FiducialResult left = tagForOpening(first, cx, cy, cz, 17, 60, 30, 1);
                    FiducialResult right = tagForOpening(first + 3, cx, cy, cz, 17, 60, 30, 1);
                    for (List<FiducialResult> tags : Arrays.asList(
                            Arrays.asList(left), Arrays.asList(right), Arrays.asList(left, right))) {
                        Limelight result = read(tags, first, mount);
                        assertEquals(Math.toDegrees(Math.atan2(x, z)),
                                result.getObservation().getTxDegrees(), 1e-8);
                        assertEquals(Math.sqrt(x * x + 1 + z * z),
                                result.getRawDistanceMeters().getAsDouble(), 1e-8);
                    }
                }
            }
        }
    }

    @Test
    public void duplicateIdsCannotCreateAMajorityAndTwoEqualGroupsAreRejected() {
        FiducialResult a = tagForOpening(42, 0, 0, 2, 0, 0, 0, 1);
        FiducialResult b = tagForOpening(43, 0.5, 0, 2, 0, 0, 0, 1);
        assertFalse(estimate(Arrays.asList(a, a, a, b), 42).hasTarget());
        assertFalse(estimate(Arrays.asList(a, b,
                tagForOpening(44, 0, 0, 2, 0, 0, 0, 1),
                tagForOpening(45, 0.5, 0, 2, 0, 0, 0, 1)), 42).hasTarget());
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

    /** Gera poses de tags a partir de uma abertura conhecida, usando a matriz Rz*Ry*Rx. */
    private FiducialResult tagForOpening(int id, double x, double y, double z,
                                         double roll, double pitch, double yaw, double area) {
        double[] memberX = {-0.1651, -0.06985, 0.06985, 0.1651};
        double mx = memberX[Math.floorMod(id - 30, 4)], my = 0.18255996, mz = -0.1427988;
        double cr = Math.cos(Math.toRadians(roll)), sr = Math.sin(Math.toRadians(roll));
        double cp = Math.cos(Math.toRadians(pitch)), sp = Math.sin(Math.toRadians(pitch));
        double cy = Math.cos(Math.toRadians(yaw)), sy = Math.sin(Math.toRadians(yaw));
        double rx = cy * cp * mx + (cy * sp * sr - sy * cr) * my
                + (cy * sp * cr + sy * sr) * mz;
        double ry = sy * cp * mx + (sy * sp * sr + cy * cr) * my
                + (sy * sp * cr - cy * sr) * mz;
        double rz = -sp * mx + cp * sr * my + cp * cr * mz;
        return tag(id, x + rx, y + ry, z + rz, roll, pitch, yaw, area);
    }
}
